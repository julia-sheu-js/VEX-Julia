// /*    Improved Autonomous with PID Control - VEX V5            
/*----------------------------------------------------------------------------*/
/*    Improved Autonomous with PID Control - VEX V5                          */
/*----------------------------------------------------------------------------*/
#include "vex.h"
#include <cmath>
bool conveyor_enabled = true;
bool reverse_on_blue = true;
double detect_wait_time = 0.3;
double reset_wait_time = 1.0;
int conspeed = 100;
using namespace vex;
// Competition and devices
competition Competition;
brain Brain;
controller Controller1;
// Motor definitions
motor frontLeft = motor(PORT13, true);
motor midLeft = motor(PORT12, true);
motor backLeft = motor(PORT11, true);
motor frontRight = motor(PORT16, false);
motor midRight = motor(PORT18, false);
motor backRight = motor(PORT20, false);
motor Intake = motor(PORT14, false);
motor intake2 = motor(PORT15, true);
// Pneumatic definitions
digital_out piston1 = digital_out(Brain.ThreeWirePort.C);
digital_out piston2 = digital_out(Brain.ThreeWirePort.B);
bool piston1State = false;
bool piston2State = false;
// Motor groups
motor_group leftWheels(frontLeft, midLeft, backLeft);
motor_group rightWheels(frontRight, midRight, backRight);
// Vision sensor
// vision::signature Vision14__SIG_1 = vision::signature(1, -3597, -2571, -3084, 7307, 9593, 8450, 3.0, 0);
// vision::signature Vision14__SIG_2 = vision::signature(2, 6845, 8937, 7891, -1053, -527, -790, 3.0, 0);
// vision Vision14 = vision(PORT14, 50, Vision14__SIG_1, Vision14__SIG_2);
/*---------------------------------------------------------------------------*/
/*                          PID Variables                                     */
/*---------------------------------------------------------------------------*/
// PID Constants
double kP = 0.15;       // Increased slightly for better response
double kI = 0.000;      // Reduced - was causing creeping
double kD = 0.13;      //15 // REDUCED from 0.35 - this is critical
double tkP = 0.15;
double tkI = 0.0;
double tkD = 0.20;      // Slightly reduced
double drivetkP = 0.0; // Reduced back - was over-correcting
double turntkP = 0.10;
// PID Variables
int desiredValue = 0;
int desiredTurnValue = 0;
int error = 0;
int prevError = 0;
int derivative = 0;
int totalError = 0;
int turnError = 0;
int turnPrevError = 0;
int turnDerivative = 0;
int turnTotalError = 0;
bool resetDriveSensors = false;
bool enableDrivePID = false;
int avgPosition = 0;
// Settling detection - loosened for stability
int lateralSettleError = 30;   // Increased from 10 - allows more tolerance
int turnSettleError = 25;      // Increased from 10 - reduces micro-adjustments
int settleTime = 70;          // Slightly increased
int settleCount = 0;
double maxAutonSpeed = 80.0;
// Motor bias for drift correction
double rightBias = 1.0;
double leftBias = 1.0;
// Minimum power threshold - with deadband
const double minPowerThreshold = 8.0;  // Increased from 5.0
const double deadbandThreshold = 15.0; // New: ignore small errors completely
void pre_auton(void) {
// Initialize and calibrate sensors here
}
/*---------------------------------------------------------------------------*/
/*                          PID Function                                      */
/*---------------------------------------------------------------------------*/
int drivePID() {
while (enableDrivePID) {
if (resetDriveSensors) {
    resetDriveSensors = false;
    frontLeft.setPosition(0, degrees);
    midLeft.setPosition(0, degrees);
    backLeft.setPosition(0, degrees);
    frontRight.setPosition(0, degrees);
    midRight.setPosition(0, degrees);
    backRight.setPosition(0, degrees);
    prevError = 0;
    totalError = 0;
    turnPrevError = 0;
    turnTotalError = 0;
    settleCount = 0;
}
// Read individual motor positions and average them
int leftMotorPosition = (frontLeft.position(degrees) + midLeft.position(degrees) + backLeft.position(degrees)) / 3;
int rightMotorPosition = (frontRight.position(degrees) + midRight.position(degrees) + backRight.position(degrees)) / 3;
// Lateral PID
avgPosition = (leftMotorPosition + rightMotorPosition) / 2;
error = desiredValue - avgPosition;
// Add integral with anti-windup
if (std::abs(error) < 200) {  // Only accumulate when close to target
    totalError += error;
} else {
    totalError = 0;
}
derivative = error - prevError;
double lateralMotorPower = (error * kP) + (totalError * kI) + (derivative * kD);
// Turning PID
int turnDiff = leftMotorPosition - rightMotorPosition;
turnError = desiredTurnValue - turnDiff;
turnDerivative = turnError - turnPrevError;
double turnMotorPower = (turnError * tkP) + (turnDerivative * tkD);
// Apply deadband - ignore very small errors
if (std::abs(error) < deadbandThreshold) {
    lateralMotorPower = 0;
}
if (std::abs(turnError) < deadbandThreshold) {
    turnMotorPower = 0;
}
// Apply motor powers
double leftPower = (lateralMotorPower + turnMotorPower) * leftBias;
double rightPower = (lateralMotorPower - turnMotorPower) * rightBias;
if (leftPower > maxAutonSpeed) leftPower = maxAutonSpeed;
if (leftPower < -maxAutonSpeed) leftPower = -maxAutonSpeed;
if (rightPower > maxAutonSpeed) rightPower = maxAutonSpeed;
if (rightPower < -maxAutonSpeed) rightPower = -maxAutonSpeed;
// Apply minimum power threshold to prevent creeping
if (std::abs(leftPower) < minPowerThreshold) leftPower = 0;
if (std::abs(rightPower) < minPowerThreshold) rightPower = 0;
// Settling detection with velocity check and early stop
if (std::abs(error) < lateralSettleError &&
    std::abs(turnError) < turnSettleError &&
    std::abs(leftWheels.velocity(percent)) < 10 &&  // Slightly loosened from 3s
    std::abs(rightWheels.velocity(percent)) < 10)
{
    settleCount++;
    // Stop motors when settled - IMMEDIATE stop, no waiting
    if (settleCount >= (settleTime / 20)) {
        // Force immediate stop
        leftWheels.stop(hold);
        rightWheels.stop(hold);
        // Reset ALL PID state variables
        totalError = 0;
        turnTotalError = 0;
        error = 0;
        turnError = 0;
        prevError = 0;
        turnPrevError = 0;
        derivative = 0;
        turnDerivative = 0;
        settleCount = 0;
        // Exit settling check - don't continue adjusting
        while (resetDriveSensors == false && enableDrivePID) {
            task::sleep(20);
        }
        continue;
    }
}
else {
    settleCount = 0;
}
// Only spin motors if power is above threshold
if (leftPower != 0 || rightPower != 0) {
    leftWheels.spin(forward, leftPower, velocityUnits::pct);
    rightWheels.spin(forward, rightPower, velocityUnits::pct);
} else {
    // If no power needed, use hold brake
    leftWheels.stop(hold);
    rightWheels.stop(hold);
}
prevError = error;
turnPrevError = turnError;
task::sleep(20);
}
// Stop motors with hold when PID is disabled
leftWheels.stop(hold);
rightWheels.stop(hold);
return 1;
}
/*---------------------------------------------------------------------------*/
/*                      Helper Functions                                      */
/*---------------------------------------------------------------------------*/
int distanceToMotorValue(double distance) {
return distance * 7.245;
}
void driveTo(double target, int timeout_ms = 3000) {
resetDriveSensors = true;
desiredValue = distanceToMotorValue(target);
desiredTurnValue = 0;
settleCount = 0;
tkP = drivetkP;
int elapsed = 0;
while (settleCount < (settleTime / 20) && elapsed < timeout_ms) {
task::sleep(20);
elapsed += 20;
}
}
int degreesToMotorDiff(double degrees) {
return degrees * (4.2285);
}
void turnTo(double target, int timeout_ms = 3000) {
resetDriveSensors = true;
desiredValue = 0;
desiredTurnValue = degreesToMotorDiff(target);
settleCount = 0;
tkP = turntkP;
int elapsed = 0;
while (settleCount < (settleTime / 20) && elapsed < timeout_ms) {
task::sleep(20);
elapsed += 20;
}
}
/*---------------------------------------------------------------------------*/
/*                      Intake Task Functions                                 */
/*---------------------------------------------------------------------------*/
int intakeDuration = 0;
bool intakeReverse = false;
bool runIntakeTask = false;
//bool runIntakeTaskStop = false;
int intakeTaskFunction() {
while (true) {
if (runIntakeTask) {
    if (intakeReverse) {
        Intake.spin(forward, conspeed, percent);
        intake2.spin(forward, conspeed, percent);
    } else {
        Intake.spin(forward, conspeed, percent);
        intake2.spin(reverse, conspeed, percent);
    }
    task::sleep(intakeDuration);
    Intake.stop();
    intake2.stop();
    runIntakeTask = false;
}
task::sleep(20);
}
return 0;
}
//int intakeTaskFunctionStop() {
//while (true) {
//if (runIntakeTaskStop) {
    //Intake.spin(reverse, conspeed, percent);
    //intake2.spin(reverse, conspeed, percent);
    //task::sleep(intakeDuration);
    //Intake.stop();
    //intake2.stop();
    //runIntakeTaskStop = false;
//}
//task::sleep(20);
//}
//return 0;
//}
void runIntake(int duration_s, bool Reverse = false) {
intakeDuration = duration_s;
intakeReverse = Reverse;
runIntakeTask = true;
}
//void runIntakeStop(int duration_s, bool Reverse = false) {
//intakeDuration = duration_s;
//intakeReverse = Reverse;
//runIntakeTask = true;
//}
/*---------------------------------------------------------------------------*/
/*                          Color Detection Task                             */
/*---------------------------------------------------------------------------*/
// int colorDetectionTask() {
// while (true) {
//  Vision14.takeSnapshot(Vision14__SIG_1);
//  if (Vision14.objectCount > 0 && conveyor_enabled && reverse_on_blue) {
//      Controller1.Screen.clearLine(2);
//      Controller1.Screen.setCursor(2, 1);
//      Controller1.Screen.print("Blue Object");
//      intake2.setVelocity(conspeed, percent);
//      intake2.spin(reverse);
//      wait(detect_wait_time, seconds);
//      intake2.stop();
//      wait(reset_wait_time, seconds);
//  } else {
//      Vision14.takeSnapshot(Vision14__SIG_2);
//      if (Vision14.objectCount > 0 && conveyor_enabled && !reverse_on_blue) {
//          Controller1.Screen.clearLine(2);
//          Controller1.Screen.setCursor(2, 1);
//          Controller1.Screen.print("Red Object");
//          intake2.setVelocity(conspeed, percent);
//          intake2.spin(forward);
//          wait(detect_wait_time, seconds);
//          intake2.stop();
//          wait(reset_wait_time, seconds);
//      }
//  }
//  task::sleep(50);
// }
// return 0;
// }
/*---------------------------------------------------------------------------*/
/*                          Autonomous Routine                                */
/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
/*                          Autonomous Routine                                */
/*---------------------------------------------------------------------------*/
void autonomous(void) {
enableDrivePID = true;
task pidTask(drivePID);
task intakeTask(intakeTaskFunction);
resetDriveSensors = true;








piston1State = !piston1State;
piston1.set(piston1State);






//-----solo auton-------------------------------------------
// Intake.setVelocity(100,percent);
// Intake.spin(forward);
// driveTo(20,1000);
// turnTo(-54.5,1000);
// maxAutonSpeed = 17.0;
// driveTo(73,1000);
// wait(2.5,sec);
// Intake.stop();
// maxAutonSpeed = 80.0;
// driveTo(-85,1000);
// maxAutonSpeed = 14.0;
// Intake.spin(forward);
// wait(0.5,sec);
// turnTo(200,1000);
// Intake.spin(forward);
// driveTo(50);
// maxAutonSpeed = 80.0;
// turnTo(80,1000);
// driveTo(105,2000);
// turnTo(50,2000);
// driveTo(-60,1000);
// intake2.setVelocity(100,percent);
// Intake.spin(forward);
// intake2.spin(forward);
// wait(4,sec);


//=============================left high and mid, WORKS


Intake.setVelocity(100,percent);
intake2.setVelocity(100,percent);
Intake.spin(forward);
driveTo(73.3,1000);


turnTo(-37,1000);
maxAutonSpeed= 17.0;
driveTo(35,1000);
piston1State = !piston1State;
piston1.set(piston1State);
wait(0.5,sec);
maxAutonSpeed = 80.0;
turnTo(-94.5,1000);
driveTo(-47,1000);
intake2.spin(forward);
Intake.stop();
wait(2,sec);
piston1State = !piston1State;
piston1.set(piston1State);




driveTo(120,1000);
wait(1,sec);
turnTo(-47,1000);
wait(0.5,sec);
driveTo(-55,1000);
intake2.setVelocity(100,percent);
Intake.spin(forward);
intake2.spin(forward);


//-------------------------------










enableDrivePID = false;
leftWheels.stop(hold);
rightWheels.stop(hold);
}
/*---------------------------------------------------------------------------*/
/*                          User Control                                      */
/*---------------------------------------------------------------------------*/
void usercontrol(void) {
enableDrivePID = false;
frontLeft.setBrake(brakeType::coast);
backLeft.setBrake(brakeType::coast);
midLeft.setBrake(brakeType::coast);
midRight.setBrake(brakeType::coast);




frontRight.setBrake(brakeType::coast);
backRight.setBrake(brakeType::coast);
// task colorSortTask(colorDetectionTask);
while (1) {
// Button Y - Toggle alliance color
if (Controller1.ButtonY.pressing()) {
    reverse_on_blue = !reverse_on_blue;
    Controller1.Screen.clearLine(0);
    Controller1.Screen.setCursor(0, 0);
    Controller1.Screen.print("Alliance: %s", reverse_on_blue ? "Red" : "Blue");
    wait(0.4, seconds);
}
// Button B - Toggle piston1 (Port A)
if (Controller1.ButtonX.pressing()) {
    piston1State = !piston1State;
    piston1.set(piston1State);
    wait(0.4, seconds);
}
// Button X - Toggle piston2 (Port B)
if (Controller1.ButtonB.pressing()) {
    piston2State = !piston2State;
    piston2.set(piston2State);
    wait(0.4, seconds);
}
// Tank drive control
double leftY = Controller1.Axis3.position(percent);
double rightX = Controller1.Axis1.position(percent);
// Deadzone
if (std::abs(leftY) < 5) leftY = 0;
if (std::abs(rightX) < 5) rightX = 0;
// Apply cubic curve with sensitivity
leftY = pow(leftY, 3) / 10000.0 * 0.7;
rightX = pow(rightX, 3) / 10000.0 * 0.25;
double leftSpeed = (leftY + rightX);
double rightSpeed = (leftY - rightX);
leftWheels.setVelocity(leftSpeed, velocityUnits::pct);
rightWheels.setVelocity(rightSpeed, velocityUnits::pct);
leftWheels.spin(forward);
rightWheels.spin(forward);
// Intake controls
if (Controller1.ButtonR1.pressing()) {
    Intake.spin(forward, conspeed, percent);
}
else if (Controller1.ButtonR2.pressing()) {
    Intake.spin(reverse, conspeed, percent);
}
else {
    Intake.stop();
}
if (Controller1.ButtonL1.pressing()) {
    intake2.spin(forward, conspeed, percent);
}
else if (Controller1.ButtonL2.pressing()) {
    intake2.spin(reverse, conspeed, percent);
}
else {
    intake2.stop();
}
wait(20, msec);
}
}
/*---------------------------------------------------------------------------*/
/*                          Main                                              */
/*---------------------------------------------------------------------------*/
int main() {
Competition.autonomous(autonomous);
Competition.drivercontrol(usercontrol);
pre_auton();
while (true) {
wait(100, msec);
}
}











































