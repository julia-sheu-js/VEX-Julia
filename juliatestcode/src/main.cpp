
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

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here
brain Brain;
controller Controller1;

// Motor definitions - set reversal once here
motor frontLeft = motor(PORT4, false);
motor midLeft = motor(PORT3, false);   // Left side - not reversed
motor backLeft = motor(PORT1, false);     // Left side - not reversed
motor frontRight = motor(PORT8, true);
motor midRight = motor(PORT6,true);
motor backRight = motor(PORT2, true);     // Right side - reve   // Right side - reversed
motor Intake = motor(PORT5, false);
// Motor groups for easier control
motor_group leftWheels(frontLeft, midLeft, backLeft);
motor_group rightWheels(frontRight, midRight, backRight);
motor_group all(frontLeft,midLeft, backLeft, frontRight, midRight, backRight);
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
        leftWheels.spinFor(reverse, abs(rotations), turns, false);
        rightWheels.spinFor(forward, abs(rotations), turns);
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
void autonomous(void) {
    enableDrivePID = true;
    vex::task PIDmod(drivePID);
    resetDriveSensors = true; 
    desiredValue = 300;
    desiredTurnValue = 600;
    vex::task::sleep(1000);
    
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

    enableDrivePID = false;
    while (1) {
        // Get joystick values
        double leftY = Controller1.Axis3.position();   // Left stick Y-axis
        double rightY = Controller1.Axis2.position();  // Right stick Y-axis
        
        // Apply deadzone (ignore small movements)
        if (abs(leftY) < 10) {
            leftY = 0;
        }
        if (abs(rightY) < 10) {
            rightY = 0;
        }
        
        // Apply smooth curve for better control (optional)
        // This gives more precision at low speeds
        if (leftY != 0) {
            leftY = (leftY > 0) ? 
                0.1 * leftY + 0.9 * pow(leftY, 2) / 100.0 :
                0.1 * leftY - 0.9 * pow(leftY, 2) / 100.0;
        }
        if (rightY != 0) {
            rightY = (rightY > 0) ? 
                0.1 * rightY + 0.9 * pow(rightY, 2) / 100.0 :
                0.1 * rightY - 0.9 * pow(rightY, 2) / 100.0;
        }
        if (Controller1.ButtonR1.pressing()){
          Intake.spin(foward, 100,percent);
        }
        else if (Controller1.ButtonR2.pressing()){
          Intake.spin(reverse, 100,percent);
        }
        else {
          Intake.stop();
        }
        // // Only update motors if joystick values changed significantly
        // if (abs(leftY - prevLeftY) > 2 || abs(rightY - prevRightY) > 2) {
        //     leftWheels.setVelocity(leftY, percent);
        //     rightWheels.setVelocity(rightY, percent);
            
        //     leftWheels.spin(forward);
        //     rightWheels.spin(forward);
            
        //     prevLeftY = leftY;
        //     prevRightY = rightY;
        // }
        
        wait(20, msec); // Prevent wasted resources
    }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
    // Set up callbacks for autonomous and driver control periods
    Competition.autonomous(autonomous);
    Competition.drivercontrol(usercontrol);
    
    // Run the pre-autonomous function
    pre_auton();
    
    // Prevent main from exiting with an infinite loop
    while (true) {
        wait(100, msec);
    }
}
