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
motor fLMotor = motor(PORT18, ratio6_1, false);//front left drive motor
motor fRMotor = motor(PORT13, ratio6_1, true);//front right drive motor
motor rLMotor = motor(PORT19, ratio6_1, false);//rear left drive motor
motor rRMotor = motor(PORT14, ratio6_1, true);//rear right drive motor

inertial inert = inertial(PORT5);

motor_group leftDrive = motor_group(fLMotor, rLMotor);
motor_group rightDrive = motor_group(fRMotor, rRMotor);
motor_group drive = motor_group(fLMotor, rLMotor, fRMotor, rRMotor);

float driveChanger = 0.75;

//smartdrive blackjack = smartdrive(leftDrive, rightDrive, inert, 12.57, 10.625, 9.5, inches, 2);

//intake intializations
motor hIntMotor = motor(PORT11, ratio18_1, true);
motor lIntMotor = motor(PORT20, ratio6_1, true);

motor_group intake = motor_group(hIntMotor, lIntMotor);
motor_group drivetake = motor_group(fLMotor, rLMotor, fRMotor, rRMotor, hIntMotor, lIntMotor);

//sensor intializations
aivision windshield = aivision(PORT6);
  //red ring
  aivision::colordesc RED1 = aivision::colordesc(1, 168, 9, 59, 28, 0.2);
  aivision::colordesc RED2 = aivision::colordesc(2, 203, 32, 95, 10, 0.2);
  aivision::colordesc RED3 = aivision::colordesc(3, 219, 46, 106, 10, 0.2);
  aivision::colordesc RED4 = aivision::colordesc(4, 245, 121, 184, 10, 0.2);
  color RedAvg = color((168 + 203 + 219 + 245)/4, (9 + 32 + 46 + 121)/4, (59 + 95 + 106 + 184)/4);

  aivision::codedesc redRing = aivision::codedesc(1, RED1, RED2, RED3, RED4);
  //blue ring

  //green goal
  aivision::colordesc GREEN0 = aivision::colordesc(5, 124, 241, 93, 10, 0.2);
  aivision::colordesc GREEN1 = aivision::colordesc(6, 101, 199, 82, 10, 0.2);

  aivision::codedesc goal = aivision::codedesc(3, GREEN0, GREEN1);
distance back = distance(PORT6);

//intake redirect intitializations
motor liftLeft = motor(PORT2, ratio18_1, true);
motor liftRight = motor(PORT9, ratio18_1, false);
motor_group lift = (liftLeft, liftRight);

//clamp initializations
bool clamp = false;
bool R1down = false;
pneumatics mogoClamp = pneumatics(Brain.ThreeWirePort.A);

//methods

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

//auton methods
void redDetect(){
  windshield.takeSnapshot(redRing, 8);
  if(windshield.objectCount > 0){
    Brain.Screen.drawRectangle(windshield.objects[0].originX, windshield.objects[0].originY, windshield.objects[0].width, windshield.objects[0].height, RedAvg);
  } else {
    Brain.Screen.print("failed");
  }
}
void approachRed(){
  redDetect();
  while(windshield.objects[0].originX != 0){
    leftDrive.spin(directionType::rev, 50, velocityUnits::pct);
    leftDrive.spin(directionType::fwd, 50, velocityUnits::pct);
  }

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
  mogoClamp.set(true);
  inert.calibrate();
  while(inert.isCalibrating() == true){
    Brain.Screen.clearLine();
    Brain.Screen.print("inertial calibrating, please wait");
  }
  Brain.Screen.clearLine();

  drive.resetPosition();
  intake.resetPosition();
  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}




//PID code


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
  //auton for mogo, preload onto, turn, intake, right side
  /*
  intake.setVelocity(100, velocityUnits::pct);
  drive.spinFor(directionType::rev, 2000, rotationUnits::deg, 50, velocityUnits::pct);
  drive.stop();
  mogoClamp.set(false);
  intake.spinFor(directionType::fwd, 1000, rotationUnits::deg, 100, velocityUnits::pct);
  drive.resetPosition();
  leftDrive.spinFor(directionType::rev, 550, rotationUnits::deg, 25, velocityUnits::pct);
  rightDrive.spinFor(directionType::rev, -750, rotationUnits::deg, 25, velocityUnits::pct);
  drive.resetPosition();
  intake.spin(directionType::fwd);
  drivetake.spinFor(directionType::fwd, 1750, rotationUnits::deg, 100, velocityUnits::pct);
  intake.stop();
  drive.spinFor(directionType::rev, 500, rotationUnits::deg, 100, velocityUnits::pct);
  drive.stop();
  drive.resetPosition();
  rightDrive.spinFor(directionType::rev, 550, rotationUnits::deg, 25, velocityUnits::pct);
  leftDrive.spinFor(directionType::rev, -350, rotationUnits::deg, 25, velocityUnits::pct);
  drive.resetPosition();
  intake.spin(directionType::fwd);
  drive.spinFor(directionType::fwd, 2700, rotationUnits::deg, 100, velocityUnits::pct);
  wait(1000, timeUnits::msec);
  intake.stop();
  */

  //programmer skills
  drive.spinFor(directionType::rev, 1200, rotationUnits::deg, 50, velocityUnits::pct);
  mogoClamp.set(false);
  intake.spinFor(directionType::fwd, 500, rotationUnits::deg, 100, velocityUnits::pct);
  drive.resetPosition();
  rightDrive.spinFor(directionType::rev, 550, rotationUnits::deg, 25, velocityUnits::pct);
  leftDrive.spinFor(directionType::rev, -750, rotationUnits::deg, 25, velocityUnits::pct);
  intake.setVelocity(100, velocityUnits::pct);
  intake.spin(directionType::fwd);
  drive.spinFor(directionType::fwd, 2000, rotationUnits::deg, 100, velocityUnits::pct);
  wait(1000, timeUnits::msec);
  drive.spinFor(directionType::fwd, 2000, rotationUnits::deg, 100, velocityUnits::pct);
  wait(500, timeUnits::msec);
  intake.stop();

  drive.resetPosition();
  drive.spinFor(directionType::rev, 1500, rotationUnits::deg, 100, velocityUnits::pct);
  drive.resetPosition();
  rightDrive.spinFor(directionType::rev, -750, rotationUnits::deg, 25, velocityUnits::pct);
  leftDrive.spinFor(directionType::rev, 550, rotationUnits::deg, 25, velocityUnits::pct);
  intake.setVelocity(100, velocityUnits::pct);
  drive.spinFor(directionType::rev, 1500, rotationUnits::deg, 100, velocityUnits::pct);
  wait(1000, timeUnits::msec);
  intake.stop();


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
    leftDrive.spin(directionType::fwd, (Controller.Axis3.value() + Controller.Axis1.value()) * driveChanger, velocityUnits::pct);
    rightDrive.spin(directionType::fwd, (Controller.Axis3.value() - Controller.Axis1.value()) * driveChanger, velocityUnits::pct);
    
  //intake code 
    //intake press
    Controller.ButtonL1.pressed(intakeSpinFor);
    Controller.ButtonL1.released(intakeStop);
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
