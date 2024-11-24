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
using namespace bancroft;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here
brain Brain;
controller Controller = controller();

//drivetrain initializations
motor fLMotor = motor(PORT18, ratio6_1, false);//front left drive motor
motor fRMotor = motor(PORT13, ratio6_1, true);//front right drive motor
motor topRMotor = motor(PORT15, ratio6_1, true);//top right motor
motor topLMotor = motor(PORT16, ratio6_1, false);//top left motor
motor rLMotor = motor(PORT19, ratio6_1, false);//rear left drive motor
motor rRMotor = motor(PORT14, ratio6_1, true);//rear right drive motor

inertial inert = inertial(PORT10);

motor_group leftDrive = motor_group(fLMotor, rLMotor, topLMotor);
motor_group rightDrive = motor_group(fRMotor, rRMotor, topRMotor);
motor_group drive = motor_group(fLMotor, rLMotor, topLMotor, fRMotor, rRMotor, topRMotor);

float driveChanger = 1.0;

//smartdrive blackjack = smartdrive(leftDrive, rightDrive, inert, 12.57, 10.625, 9.5, inches, 2);

//intake intializations
motor hIntMotor = motor(PORT2, ratio18_1, true);
motor lIntMotor = motor(PORT20, ratio6_1, true);

motor_group intake = motor_group(hIntMotor, lIntMotor);
motor_group drivetake = motor_group(fLMotor, rLMotor, topLMotor, fRMotor, rRMotor, topRMotor, hIntMotor, lIntMotor);


  enum VisionSignatures {
    mobileGoal,
    redRing,
    blueRing
  };

  // all object descriptions for game elements
  aivision::aiobjdesc mobileGoalObj = aivision::aiobjdesc(VisionSignatures::mobileGoal);
  aivision::aiobjdesc redRingObj = aivision::aiobjdesc(VisionSignatures::redRing);
  aivision::aiobjdesc blueRingObj = aivision::aiobjdesc(VisionSignatures::blueRing);

//sensor intializations
aivision windshieldRight = aivision(PORT9, aivision::ALL_AIOBJS);
aivision windshieldLeft = aivision(PORT3, aivision::ALL_AIOBJS);

class aivisionUtil{
  public:
  //aivision properties
    float aiXFov;
    float aiYFov;

    int xRes;
    int yRes; 

    float camDist;

    float xResFovRatio;
    float yResFovRatio;

    float camCenterDist; 

    float thetaOne;//angle from left camera
    float thetaTwo;//angle from right camera
    
    float thetaTrueOne;
    float thetaTrueTwo;
    float thetaTrue;

    float xOne;//distance from left camera 
    float xTwo;//distance from right camera

    float xTrueOne;
    float xTrueTwo;
    float xTrue;

    float xOnePix;
    float xTwoPix;

    float absolutePos;//position from center of robot


  aivisionUtil (float aiXFov, float aiYFov, int xRes, int yRes, float camDist){
    this->aiXFov = aiXFov; 
    this->aiYFov = aiYFov;
    this->xRes = xRes;
    this->yRes = yRes;
    this->camDist = camDist;
    


    xResFovRatio = aiXFov / xRes;
    yResFovRatio = aiYFov / yRes;

    camCenterDist = camDist / 2; 
  }
  float aiDistance(aivision::object lefty, aivision::object righty){
    aivision::object objLeft = lefty;
    aivision::object objRight = righty;

    if(lefty.centerX > xRes/2){
      xOnePix = lefty.centerX - xRes/2;
    } else if(lefty.centerX < xRes/2){
      xOnePix = -lefty.centerX + xRes/2;
    } else {
      xOnePix = 0;
    }

    if(righty.centerX > xRes/2){
      xTwoPix = righty.centerX - xRes/2;
    } else if(righty.centerX < xRes/2){
      xTwoPix = -righty.centerX + xRes/2;
    } else {
      xTwoPix = 0;
    }

    thetaOne = (aiXFov/xRes) * xOnePix;
    thetaTwo = (aiXFov/xRes) * xTwoPix;

    xOne = (xRes/aiXFov) * thetaOne;
    xTwo = (xRes/aiXFov) * thetaTwo;

    xTrueOne = xOne - (camCenterDist/2);
    xTrueTwo = xTwo - (camCenterDist/2);
    xTrue = (xTrueOne + xTrueTwo)/2;

    thetaTrueOne = xTrue/(xOne * thetaOne);
    thetaTrueTwo = xTrue/(xTrue * thetaTwo);
    thetaTrue = (thetaTrueOne + thetaTrueTwo)/2;

    absolutePos = xTrue/(tan(thetaTrue));
  
    return absolutePos;
  }

  float aiRotation(aivision::object lefty, aivision::object righty){
    aivision::object objLeft = lefty;
    aivision::object objRight = righty;

    if(lefty.centerX > xRes/2){
      xOnePix = lefty.centerX - xRes/2;
    } else if(lefty.centerX < xRes/2){
      xOnePix = -lefty.centerX + xRes/2;
    } else {
      xOnePix = 0;
    }

    if(righty.centerX > xRes/2){
      xTwoPix = righty.centerX - xRes/2;
    } else if(righty.centerX < xRes/2){
      xTwoPix = -righty.centerX + xRes/2;
    } else {
      xTwoPix = 0;
    }

    thetaOne = (aiXFov/xRes) * xOnePix;
    thetaTwo = (aiXFov/xRes) * xTwoPix;

    xOne = (xRes/aiXFov) * thetaOne;
    xTwo = (xRes/aiXFov) * thetaTwo;

    xTrueOne = xOne - (camCenterDist/2);
    xTrueTwo = xTwo - (camCenterDist/2);
    xTrue = (xTrueOne + xTrueTwo)/2;

    thetaTrueOne = xTrue/(xOne * thetaOne);
    thetaTrueTwo = xTrue/(xTrue * thetaTwo);
    thetaTrue = (thetaTrueOne + thetaTrueTwo)/2;

    if(xOne == xTwo){
      thetaTrue = 0;
    }

    return thetaTrue;
  } 

};

//intake redirect intitializations
motor liftLeft = motor(PORT2, ratio18_1, true);
motor liftRight = motor(PORT9, ratio18_1, false);
motor_group lift = (liftLeft, liftRight);

//clamp initializations
bool clamp = false;
bool doinker = false;
bool Adown = false;
bool R1down = false;
pneumatics mogoClamp = pneumatics(Brain.ThreeWirePort.A);
pneumatics doink = pneumatics(Brain.ThreeWirePort.B);

//methods
  //intake methods
void intakeSpinFor(){
  intake.spin(directionType::fwd, 100, velocityUnits::pct);
  //kamalaPrint();
}
void intakeSpinAga(){
  intake.spin(directionType::rev, 100, velocityUnits::pct);
  //kamalaPrint();
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
  //xylo1Print();
  R1down = true;
}
void unclamped(){
  R1down = false;
}
  //doink methods
void doinked(){
  //mogoClamp.set(true);
  
  if(Adown == false){
    doink.set(doinker);
    doinker = !doinker;
  }
  //xylo1Print();
  Adown = true;
}
void undoink(){
  Adown = false;
}

  //redirect methods
void intakeLift(){
  lift.spin(directionType::fwd, 100, velocityUnits::pct);
  //xyloPrint();
}
void intakeLiftBack(){
  lift.spin(directionType::rev, 100, velocityUnits::pct);
  //xyloPrint();
}
void intakeDownOne(){
  lift.setStopping(hold);
  lift.stop();
}
void intakeDownTwo(){
  lift.setStopping(coast);
  lift.stop();
}

PID rightSidePID = PID(0.8, 0, 0.6, 2, 2, 200);
PID leftSidePID = PID(0.8, 0, 0.6, 2, 2, 200);
float L = 11.25;
float c = 2 * M_PI * (3.25/2);

void moveSides(float rightSideDistance, float leftSideDistance) {
  drive.resetPosition();
  rightSideDistance += c * rightDrive.position(rev);
  leftSideDistance += c * leftDrive.position(rev);
  rightSidePID.pidReset();
  leftSidePID.pidReset();
  do {
    rightDrive.spin(fwd, rightSidePID.pidControl(rightSideDistance, c * rightDrive.position(rev)), volt);
    leftDrive.spin(fwd, leftSidePID.pidControl(leftSideDistance, c * leftDrive.position(rev)), volt);
    wait(10, msec);
  } while(!(rightSidePID.isSettled() && leftSidePID.isSettled()));
  rightDrive.stop(coast);
  leftDrive.stop(coast);
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
  drive.resetPosition();
  intake.resetPosition();

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}




//P-controller code, p only cause its not a steady state operation
  //PD controller future? how much do you accelerate toward it. autonomous
  //D for driver makes controller not linear, harder to drive

/*
  currentRotZ = current inertial heading
  fwdTarget = joystick value mapped from -1.0 to 1.0
  fwdConstant = experimentally determined "how fast can you increment per loop time"
  rotZIncrement = experimentally determined "what is multiplied by the joystick value of turning and added to your rotz target"
  kp * 90 = normal PID constant
  

  citation: Mr. Harrington, based off code he wrote for an Arduino-based puppy bot
  "https://github.com/BancroftSchoolOpenSource/ArduinoClassRobot/blob/main/ArduinoClassRobot.ino"

  citation: Adam Vining-Recklitis, based off code he wrote for a robot last year
  "https://github.com/BancroftRoboDogs/RobodogsCode_2023_03/blob/main/test/src/main.cpp"
  thank you especially to Adam, for allowing me to work off his stuff, love ya buddy
*/

aivisionUtil windshield = aivisionUtil(74, 63, 320, 240, 10.5);

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void){ 
  windshieldLeft.modelDetection(true);
  windshieldRight.modelDetection(true);

  intake.setVelocity(100,velocityUnits::pct);
  drive.resetPosition();
  int slotter = 4;


  // //RIGHT SIDE
  // //drive towards mobile goal
  // drive.spinFor(directionType::rev, 900, rotationUnits::deg, 50, velocityUnits::pct);
  // drive.stop();
  // //clamp goal
  // mogoClamp.set(false);
  // //load preload onto goal
  // intake.spinFor(directionType::fwd, 1000, rotationUnits::deg, 100, velocityUnits::pct);
  // drive.resetPosition();
  // //turn right
  // leftDrive.spinFor(directionType::rev, 225, rotationUnits::deg, 25, velocityUnits::pct);
  // rightDrive.spinFor(directionType::rev, -400, rotationUnits::deg, 25, velocityUnits::pct);
  // drive.resetPosition();
  // //intake ring
  // intake.spin(directionType::fwd);
  // drive.spinFor(directionType::fwd, 1500, rotationUnits::deg, 100, velocityUnits::pct);
  // wait(375, timeUnits::msec);
  // //drive back towards tower
  // intake.stop();
  // drive.spinFor(directionType::rev, 1650, rotationUnits::deg, 100, velocityUnits::pct);
  // drive.resetPosition();
  // wait(1000, timeUnits::msec);
  // intake.stop();
/*
  //RIGHT SIDE W/Intertial
  if (slotter == 1){
    //move towards goal
    moving.moveTo(24);
    //clamp
    mogoClamp.set(false);
    //intake preload onto goal, keep intake running
    intake.spin(directionType::fwd);
    //turn towards ring stack
    turning.turnTo(90);
    //move into rings
    moving.moveTo(24);
  }

  // //LEFT SIDE
  // //drive towards mobile goal
  // drive.spinFor(directionType::rev, 900, rotationUnits::deg, 50, velocityUnits::pct);
  // drive.stop();
  // //clamp goal
  // mogoClamp.set(false);
  // //load preload onto goal
  // intake.spinFor(directionType::fwd, 1000, rotationUnits::deg, 100, velocityUnits::pct);
  // drive.resetPosition();
  // //turn left
  // rightDrive.spinFor(directionType::rev, 225, rotationUnits::deg, 25, velocityUnits::pct);
  // leftDrive.spinFor(directionType::rev, -400, rotationUnits::deg, 25, velocityUnits::pct);
  // drive.resetPosition();
  // //intake ring
  // intake.spin(directionType::fwd);
  // drive.spinFor(directionType::fwd, 1500, rotationUnits::deg, 100, velocityUnits::pct);
  // wait(375, timeUnits::msec);
  // //drive back towards tower
  // intake.stop();
  // drive.spinFor(directionType::rev, 1650, rotationUnits::deg, 100, velocityUnits::pct);
  // drive.resetPosition();
  // wait(1000, timeUnits::msec);
  // intake.stop();

  //LEFT SIDE W/Intertial
  if (slotter == 2){
    drive.resetPosition();
  //move towards goal
  moving.moveTo(-42);
  //clamp
  mogoClamp.set(false);
  //turn towards ring stack
  turning.turnTo(-130);
  //intake preload onto goal, keep intake running
  intake.spin(directionType::fwd);
  //move into rings
  drive.resetPosition();
  moving.moveTo(24);
  turning.turnTo(-90);
  moving.moveTo(24);

  }
  */
  //EXPERIEMENTAL AUTON WITH AI-BASED PID
  if(slotter == 3){
    while(true){

      printf("%d, %d", windshieldLeft.installed(), windshieldRight.installed());

      windshieldLeft.takeSnapshot(redRingObj, 8);
      windshieldRight.takeSnapshot(redRingObj, 8);

      wait(10, timeUnits::msec);
      Brain.Screen.clearScreen();
      if(windshieldLeft.objectCount < 1){
        Brain.Screen.printAt(0, 30, "Left: I can't see shit homie");
      } else {
        Brain.Screen.printAt(0, 30, "Left: I can see: %f", (windshield.aiRotation(windshieldLeft.objects[0], windshieldRight.objects[0]))*10);
      }
    }
  }

  if(slotter == 4){
    
    moveSides(90/9, -90/9);

    }
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
    Brain.Screen.print("auton");

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
    //doink code
    Controller.ButtonA.pressed(doinked);
    Controller.ButtonA.released(undoink);
    //redirect code
    Controller.ButtonR1.pressed(intakeLift);
    Controller.ButtonR2.pressed(intakeLiftBack);
    Controller.ButtonR1.released(intakeDownOne);
    Controller.ButtonR2.released(intakeDownTwo);
    //avgTemp();

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
