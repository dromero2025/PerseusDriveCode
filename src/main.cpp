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
motor fLMotor = motor(PORT18, ratio36_1, false);//front left drive motor
motor fRMotor = motor(PORT13, ratio36_1, true);//front right drive motor
motor rLMotor = motor(PORT19, ratio36_1, false);//rear left drive motor
motor rRMotor = motor(PORT14, ratio36_1, true);//rear right drive motor

inertial inert = inertial(PORT5);

motor_group leftDrive = motor_group(fLMotor, rLMotor);
motor_group rightDrive = motor_group(fRMotor, rRMotor);

//smartdrive blackjack = smartdrive(leftDrive, rightDrive, inert, 12.57, 10.625, 9.5, inches, 2);

//intake intializations
motor hIntMotor = motor(PORT11, ratio18_1, true);
motor lIntMotor = motor(PORT20, ratio18_1, true);
double intakeRot = 0.0;

motor_group intake = motor_group(hIntMotor, lIntMotor);

optical chucker = optical(PORT8);
double redLow = 10.0;
double redHigh = 25.0;

//sensor intializations
distance front = distance(PORT5);
//distance back = distance(PORT3);
aivision backupCam = aivision(PORT4);

//intake redirect intitializations
motor liftLeft = motor(PORT2, ratio18_1, true);
motor liftRight = motor(PORT2, ratio18_1, false);
motor_group lift = (liftLeft, liftRight);

//clamp initializations
bool clamp = false;
bool R1down = false;
pneumatics mogoClamp = pneumatics(Brain.ThreeWirePort.A);

//climb initializations
motor climb1 = motor(PORT11, ratio6_1, true);
motor climb2 = motor(PORT12, ratio6_1, false);

distance groundSense = distance(PORT13);



//methods
void chuck(){
  if(chucker.hue() >= redLow || chucker.hue() <= redHigh){
    Brain.Screen.clearLine();
    Brain.Screen.print("Red Detected");
    intake.spin(directionType::rev, 100, velocityUnits::pct);
  } else if(chucker.hue() < redLow || chucker.hue() > redHigh){

  }
}

  //intake methods
void intakeSpinFor(){
  intake.spin(directionType::fwd, 100, velocityUnits::pct);
}
void intakeSpinAga(){
  intake.spin(directionType::rev, 100, velocityUnits::pct);
}
void intakeStop(){
  intake.stop();
}
  //clamp methods
void clamped(){
  //mogoClamp.set(true);
  
  if(R1down == false){
    mogoClamp.set(clamp);
    clamp = !clamp;
  }
  R1down = true;
}
void unclamped(){
  R1down = false;
}

  //redirect methods
void intakeLift(){
  lift.spin(directionType::fwd, 100, velocityUnits::pct);
}
void intakeLiftBack(){
  lift.spin(directionType::rev, 100, velocityUnits::pct);
}
void intakeDownOne(){
  lift.setStopping(hold);
  lift.stop();
}
void intakeDownTwo(){
  lift.setStopping(coast);
  lift.stop();
}


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
  chucker.setLight(ledState::on);
  chucker.setLightPower(75, percentUnits::pct);
  

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
    chucker.objectDetected(chuck);
    //outtake press
    Controller.ButtonDown.pressed(intakeSpinAga);
    Controller.ButtonDown.released(intakeStop);

    //clamp code
    Controller.ButtonL2.pressed(clamped);
    Controller.ButtonL2.released(unclamped);
    //redirect code
    Controller.ButtonR1.pressed(intakeLift);
    Controller.ButtonR2.pressed(intakeLiftBack);
    Controller.ButtonR1.released(intakeDownOne);
    Controller.ButtonR2.released(intakeDownTwo);

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
