

/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       juliamacbook                                              */
/*    Created:      9/10/2025, 4:59:41 PM                                     */
/*    Descripti on:  V5 projmect - Corrected Tank Drive                        */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include <cmath>
#include "global.h"
using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here
brain Brain;
controller Controller1;

// Motor definitions - set reversal once here
motor frontLeft = motor(PORT4, true);
motor midLeft = motor(PORT3, true);   // Left side - not reversed
motor backLeft = motor(PORT1, true);     // Left side - not reversed
motor frontRight = motor(PORT8, false); 
motor midRight = motor(PORT6,false);
motor backRight = motor(PORT2);     // Right side - reve   // Right side - reversed
motor Intake = motor(PORT5, false);

int conspeed = 80;

// Motor groups for easier control
motor_group leftWheels(frontLeft, midLeft, backLeft);
motor_group rightWheels(frontRight, midRight, backRight);
motor_group all(frontLeft,midLeft, backLeft, frontRight, midRight, backRight);
vision::signature Vision14__SIG_1 = vision::signature (0, 0, 0, 0, 0, 0, 0, 0, 0); // BLUE
vision::signature Vision14__SIG_2 = vision::signature (0, 0, 0, 0, 0, 0, 0, 0, 0); // RED
vision Vision14 = vision (PORT14, 50, Vision14__SIG_1, Vision14__SIG_2);

// Function to drive straight forward/backward
void vert(double velocity, double rotations) {
    all.setVelocity(velocity, percent);
    all.spinFor(forward, rotations, turns);
}

// Function to turn/pivot (positive rotations = turn right)
void pivot(double velo, double rotations) {
    // For turning: left wheels forward, right wheels backward (or vice versa)
    leftWheels.setVelocity(velo, percent);
    rightWheels.setVelocity(velo, percent);
    
    if (rotations > 0) {
        // Turn right: left forward, right backward
        leftWheels.spinFor(forward, rotations, turns, false);
        rightWheels.spinFor(reverse, rotations, turns);
    } else {
        // Turn left: left backward, right forward
        leftWheels.spinFor(reverse, std::abs(rotations), turns, false);
        rightWheels.spinFor(forward, std::abs(rotations), turns);
    }
}

// Function to stop all motors
void stop() {
    all.stop();
}

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
    // All activities that occur before the competition starts
    // Example: clearing encoders, setting servo positions, ...
}

/*---------------------------------------------------------------------------*/
/*                              Autonomous Task                              */
/*---------------------------------------------------------------------------*/
double kP = 0.0;
double kI = 0.0;
double kD = 0.0;
double tkP = 0.0;
double tkI = 0.0;
double tkD = 0.0;
// auton settings
int desiredValue = 200; // want motor to go 200 deg
int desiredTurnValue = 0;
int error; // sensor value (current) - desired value : position 
int prevError = 0; // position 20 mill sec ago
int derivative; // error - preverror : speed
int totalError = 0; //totalError = totalError + error

int turnError; // sensor value (current) - desired value : position 
int turnPrevError = 0; // position 20 mill sec ago
int turnDerivative; // error - preverror : speed
int turnTotalError = 0; //totalError = totalError + error

bool resetDriveSensors = false;

bool enableDrivePID = true;
int avgPosition;

int drivePID(){
    while(enableDrivePID){
        if (resetDriveSensors){
            resetDriveSensors = false;
            leftWheels.setPosition(0,degrees);
            rightWheels.setPosition(0,degrees);
        }
        int leftMotorPosition = leftWheels.position(degrees);
        int rightMotorPosition = rightWheels.position(degrees);
        //////////////////////////////////////////////////////
        //lateral movement PID
        //////////////////////////////////////////////////////
        
        avgPosition = -(leftMotorPosition + rightMotorPosition)/2;
        error = avgPosition - desiredValue;
        derivative = error - prevError;
        // velocity-> position -> absement
        //absement is position*time, integral converts position to absement, detect if         //object is going too slow
        //integral
        //totalError +=error;
        //volt just until 12, unlike percent
        double lateralMotorPower = error * kP + derivative * kD;
        //////////////////////////////////////////////////////
        //turning movement PID
        //////////////////////////////////////////////////////
        int turnDiff = leftMotorPosition - rightMotorPosition;
        
        turnError = turnDiff - desiredTurnValue;
        turnDerivative = turnError - turnPrevError;
        // velocity-> position -> absement
        //absement is position*time, integral converts position to absement, detect if         //object is going too slow
        //integral
        //turnTotalError += turnError;
        //volt just until 12, unlike percent
        double turnMotorPower = turnError * tkP + turnDerivative * tkD;
        /////////////////////////////////////////////////////
        leftWheels.spin(forward, lateralMotorPower + turnMotorPower, voltageUnits :: volt);
        rightWheels.spin(forward, lateralMotorPower - turnMotorPower, voltageUnits :: volt);
        // code
        turnPrevError = turnError;
        prevError = error;
        vex::task::sleep(20);
        
    
    }
    
    return 1;
    

}
int colorDetectionTask() {
    
    while (true) {
    // Blue detection
    Vision14.takeSnapshot(Vision14__SIG_1);
    if (Vision14.objectCount > 0 && conveyor_enabled && reverse_on_blue) {
      Controller1.Screen.clearLine(2);
      Controller1.Screen.setCursor(2, 1);
      Controller1.Screen.print("Blue Object");
      Intake.setVelocity(conspeed, percent);
      Intake.spin(reverse);
      wait(detect_wait_time, seconds);
      Intake.stop();
      wait(reset_wait_time, seconds);
    }
    // Red detection
    
    else if (Vision14.takeSnapshot(Vision14__SIG_2), Vision14.objectCount > 0 && conveyor_enabled && !reverse_on_blue) {
      Controller1.Screen.clearLine(2);
      Controller1.Screen.setCursor(2, 1);
      Controller1.Screen.print("Red Object");
      Intake.setVelocity(conspeed, percent);
      Intake.spin(forward);
      wait(detect_wait_time, seconds);
      Intake.stop();
      wait(reset_wait_time, seconds);
    }

    vex::task::sleep(50); // Prevent CPU overload
  }

  return 0;
}


void autonomous(void) {
    enableDrivePID = true;
    vex::task PIDmod(drivePID);
    // going straight back to first blocks
    resetDriveSensors = true; 
    desiredValue = 300;
    desiredTurnValue = 600;
    vex::task::sleep(1000);

    // going to matchload
    resetDriveSensors = true;

    
    enableDrivePID = false;
    PIDmod.stop();
    Controller1.Screen.setCursor(1,1);
    Controller1.Screen.print(desiredValue);
    Controller1.Screen.setCursor(2,1);
    Controller1.Screen.print(avgPosition);
    Controller1.Screen.setCursor(3,1);
    Controller1.Screen.print(error);
    
    // Example autonomous routine
    //vert(50, 2);       // Drive forward at 50% speed for 2 rotations
    //wait(500, msec);   // Wait half a second
    //pivot(30, 1);      // Turn right at 30% speed for 1 rotation
    //wait(500, msec);
    //vert(50, 1);       // Drive forward 1 more rotation
    //stop();
}
/*---------------------------------------------------------------------------*/
/*                              User Control Task                            */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
    // Variables to store previous joystick valuessawq  1`12qwsdza
    Controller1.Screen.clearLine(0);
    if (reverse_on_blue){
        Controller1.Screen.setCursor(0,0);
        Controller1.Screen.print("Alliance: rEd");
    }
    else {
        Controller1.Screen.setCursor(0,0);
        Controller1.Screen.print("Alliance: BLuE");
    }
    Controller1.Screen.setCursor(1,1);
    Controller1.Screen.print("Conveyeyor: %s", conveyor_enabled);
    enableDrivePID = false;
    while (1) {
        // Get joystick values
        double leftY = Controller1.Axis3.position(percent);
        double rightX = Controller1.Axis1.position(percent);   
  
        int leftSpeed = leftY + rightX;
        int rightSpeed = leftY - rightX;


        leftWheels.setVelocity(leftSpeed, velocityUnits::pct);
        rightWheels.setVelocity(rightSpeed, velocityUnits::pct);


        leftWheels.spin(forward);
        rightWheels.spin(forward);

        // Apply deadzone (ignore small movements)
        if (std::abs(leftY) < 5) {
            leftY = 0;
        }
        if (std::abs(rightX) < 5) {
            rightX = 0;
        }
        
        // Apply smooth curve for better control (optional)
        // This gives more precision at low speeds
        leftY = pow(leftY,3)/10000;
        rightX = pow(leftY,3)/10000;
       
        if (Controller1.ButtonR1.pressing()){
          Intake.spin(forward, 80,percent);
        }
        else if (Controller1.ButtonR2.pressing()){
          Intake.spin(reverse, 80,percent);
        }
        else {
          Intake.stop();
        }

        
        leftWheels.spin(forward);
        rightWheels.spin(forward);
        wait(20, msec); // Prevent wasted resources
    }
}

//
// Main will set up the competition functions and callbacks.
//
int main() 
{
    // Set up callbacks for autonomous and driver control periods
    Competition.autonomous(autonomous);
    Competition.drivercontrol(usercontrol);
    

  // Start the color detection as a background process
    vex::task colorSortTask(colorDetectionTask);

    while (true) {
    // Example: toggle conveyor on/off using controller button A
        if (Controller1.ButtonA.pressing()) {
            conveyor_enabled = !conveyor_enabled;
            Controller1.Screen.clearScreen();
            Controller1.Screen.print("Conveyor: %s", conveyor_enabled ? "ON" : "OFF");
    wait(0.4, seconds); }

    // Run the pre-autonomous function
    pre_auton();
    
    // Prevent main from exiting with an infinite loop
    while (true) {
        wait(100, msec);
    }
}
