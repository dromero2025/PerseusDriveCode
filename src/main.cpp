/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       demetrieromero                                            */
/*    Created:      9/19/2024, 1:08:29 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here
brain Brain;
controller Controller = controller();

//drivetrain initializations
motor fLMotor = motor(PORT1, ratio36_1, false);//front left drive motor
motor fRMotor = motor(PORT3, ratio36_1, true);//front right drive motor
motor rLMotor = motor(PORT2, ratio36_1, false);//rear left drive motor
motor rRMotor = motor(PORT4, ratio36_1, true);//rear right drive motor

inertial inert = inertial(PORT5);

motor_group leftDrive = motor_group(fLMotor, rLMotor);
motor_group rightDrive = motor_group(fRMotor, rRMotor);

//smartdrive blackjack = smartdrive(leftDrive, rightDrive, inert);

//intake intializations
motor hIntMotor = motor(PORT15, ratio18_1, false);
motor lIntMotor = motor(PORT14, ratio18_1, false);

motor_group intake = motor_group(hIntMotor, lIntMotor);

void intakeSpinFor(){
  intake.spin(directionType::fwd, 100, velocityUnits::pct);
}
void intakeSpinAga(){
  intake.spin(directionType::rev, 100, velocityUnits::pct);
}
void intakeStop(){
  intake.stop();
}


//clamp initializations

digital_out mogoClamp = digital_out(Brain.ThreeWirePort.A);
bool mogoCont = false;

void clamped(){
  mogoCont != mogoCont;
  mogoClamp.set(mogoCont);
}

//climb initializations



/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...

  mogoClamp.set(mogoCont);

}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  // User control code here, inside the loop
  while (1) {

  //drivetrain code
    leftDrive.spin(directionType::fwd, Controller.Axis3.value() + Controller.Axis1.value(), velocityUnits::pct);
    rightDrive.spin(directionType::fwd, Controller.Axis3.value() - Controller.Axis1.value(), velocityUnits::pct);
    
  //intake code 
    //intake press
    Controller.ButtonL1.pressed(intakeSpinFor);
    Controller.ButtonL1.released(intakeStop);
    //outtake press
    Controller.ButtonDown.pressed(intakeSpinAga);
    Controller.ButtonDown.released(intakeStop);

    /*
    //Code that runs intake correctly, but drive and intake coudln't run simultaneously
    while(Controller.ButtonL1.pressing()){
      intake.spin(directionType::fwd, 100, velocityUnits::pct);
    }
    intake.stop();
    while(Controller.ButtonL2.pressing()){
      intake.spin(directionType::rev, 100, velocityUnits::pct);
    }
    intake.stop();
    */

   //clamp code
    Controller.ButtonR1.pressed(clamped);

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
