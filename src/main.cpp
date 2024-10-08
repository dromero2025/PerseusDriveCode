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
motor fLMotor = motor(PORT12, ratio36_1, true);//front left drive motor
motor fRMotor = motor(PORT19, ratio36_1, true);//front right drive motor
motor rLMotor = motor(PORT13, ratio36_1, false);//rear left drive motor
motor rRMotor = motor(PORT20, ratio36_1, true);//rear right drive motor

inertial inert = inertial(PORT1);

motor_group leftDrive = motor_group(fLMotor, rLMotor);
motor_group rightDrive = motor_group(fRMotor, rRMotor);

//smartdrive blackjack = smartdrive(leftDrive, rightDrive, inert, 12.57, 10.625, 9.5, inches, 2);

//intake intializations
motor hIntMotor = motor(PORT10, ratio18_1, false);
motor lIntMotor = motor(PORT9, ratio18_1, false);

motor_group intake = motor_group(hIntMotor, lIntMotor);

optical chucker = optical(PORT8);

//sensor intializations
distance front = distance(PORT4);
distance back = distance(PORT3);
vision backupCam = vision(PORT2);
aivision frontCam = aivision(PORT5);

void intakeSpinFor(){
  intake.spin(directionType::fwd, 100, velocityUnits::pct);
}
void intakeSpinAga(){
  intake.spin(directionType::rev, 100, velocityUnits::pct);
}
void intakeStop(){
  intake.stop();
}

//intake redirect intitializations 

motor liftLeft = motor(PORT6, ratio18_1, true);
motor liftRight = motor(PORT7, ratio18_1, false);
motor_group lift = motor_group(liftLeft, liftRight);
void intakeLift(){
  lift.spin(directionType::fwd, 50, velocityUnits::pct);
  
}
void intakeDown(){
  lift.spin(directionType::rev, 50, velocityUnits::pct);
  
}
void intakeLiftStop(){
  lift.setStopping(brakeType::hold);
  lift.stop();
}
void intakeLiftRelease(){
  lift.setStopping(brakeType::coast);
  lift.stop();
}


//clamp initializations
bool clamp = false;
bool R1down = false;
pneumatics mogoClamp = pneumatics(Brain.ThreeWirePort.B);

void clamped(){
  //mogoClamp.set(true);
  
  if(R1down == false){
    mogoClamp.set(clamp);
    clamp = !clamp;
  }
  R1down = true;
}
void R1released(){
  R1down = false;
}

//climb initializations
motor climb1 = motor(PORT11, ratio6_1, true);
motor climb2 = motor(PORT12, ratio6_1, false);

distance groundSense = distance(PORT13);

//nonspecific sensor initializations
aivision collinsBetterEye = aivision(PORT21);

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

    //clamp code
    Controller.ButtonL2.pressed(clamped);
    Controller.ButtonL2.released(R1released);
    //redirect code
    Controller.ButtonR1.pressed(intakeLift);
    Controller.ButtonR1.released(intakeLiftStop);
    Controller.ButtonR2.pressed(intakeDown);
    Controller.ButtonR2.released(intakeLiftRelease);

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
