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
motor topRMotor = motor(PORT15, ratio6_1, true);//top right motor
motor topLMotor = motor(PORT16, ratio6_1, false);//top left motor
motor rLMotor = motor(PORT19, ratio6_1, false);//rear left drive motor
motor rRMotor = motor(PORT14, ratio6_1, true);//rear right drive motor
motor hSMotor = motor(PORT, ratio6_1,true)//High Steaks Motor

inertial inert = inertial(PORT10);

motor_group leftDrive = motor_group(fLMotor, rLMotor, topLMotor);
motor_group rightDrive = motor_group(fRMotor, rRMotor, topRMotor);
motor_group drive = motor_group(fLMotor, rLMotor, topLMotor, fRMotor, rRMotor, topRMotor);

float driveChanger = 1.0;

float turnChanger(float x){
  return (1/10000) * (pow(x, 3));
}

//smartdrive blackjack = smartdrive(leftDrive, rightDrive, inert, 12.57, 10.625, 9.5, inches, 2);

//intake intializations
motor hIntMotor = motor(PORT2, ratio18_1, true);
motor lIntMotor = motor(PORT20, ratio6_1, true);

motor_group intake = motor_group(hIntMotor, lIntMotor);
motor_group drivetake = motor_group(fLMotor, rLMotor, topLMotor, fRMotor, rRMotor, topRMotor, hIntMotor, lIntMotor);


distance clamper = distance(PORT3);
distance stakesSensor =  distance(PORT10);


class PID{
 public:
   double inertVal=0.0;
   double motorEncoderLeft=0.0;
   double motorEncoderRight=0.0;
   double error=0.0;
   double errorL=0.0;
   double errorR=0.0;
   double kP=0.0;
   double kI=0.0;
   double kD=0.0;
   double integral=0.0;
   double derivative=0.0;
   double prevError=0.0;
   double output=0.0;

  PID(){}
  PID(double kP, double kI, double kD){
    this->kP=kP;
    this->kI=kI;
    this->kD=kD;
  }
  void turnTo(double target){
    int settle = 0;
    inertVal=inert.rotation(degrees);
    error=target-inertVal;
    while(fabs(error)>0.1){
      inertVal=inert.rotation(degrees);
      error=target-inertVal;
      integral=integral+error;
      // if(error==0 range){
      //   integral=0;
      // } implement fastest route
      derivative=error-prevError;
      prevError=error;
      output=error*kP+integral*kI+derivative*kD;
      leftDrive.spin(forward, output, volt);
      rightDrive.spin(reverse, output, volt);
      printf("turnto %f \n",error);
      if(error<5&&error>(-5)){
        settle++;
      }
      if(settle>50){
        error=0;
        leftDrive.stop();
        rightDrive.stop();
      }
      wait(50,msec);
    }
  }
  void moveTo(double target){
    int settle = 0;
    motorEncoderLeft=fLMotor.position(degrees)*(3.25*M_PI)/360;
    motorEncoderRight=fRMotor.position(degrees)*(3.25*M_PI)/360;
    errorL=target-motorEncoderLeft;
    errorR=target-motorEncoderRight;
    error=(errorL+errorR)/2;
    while(fabs(error)>0.1){
      motorEncoderLeft=fLMotor.position(degrees)*(3.25*M_PI)/360;
      motorEncoderRight=fRMotor.position(degrees)*(3.25*M_PI)/360;
      errorL=target-motorEncoderLeft;
      errorR=target-motorEncoderRight;
      error=(errorL+errorR)/2;
      integral=integral+error;
      // if(error==0 range){
      //   integral=0;
      // } implement fastest route
      derivative=error-prevError;
      prevError=error;
      output=error*kP+integral*kI+derivative*kD;
      leftDrive.spin(forward, output, volt);
      rightDrive.spin(forward, output, volt);
      printf("moveto: %f \n",error);
      if(error<5 && error>(-5)){
        settle++;
      }
      if(settle>50){
        error=0;
        leftDrive.stop();
        rightDrive.stop();
        leftDrive.resetPosition();
        rightDrive.resetPosition();
      }
            wait(10,msec);
    }
    leftDrive.resetPosition();
    rightDrive.resetPosition();
  }
};

PID turning = PID(0.4, 0.0, 0.65);
PID moving = PID(0.6, 0.0, 0.5);

float mapTo180(float angle) {
  return angle - 360 * (int(angle) / 180 - int(angle) / 360);
}
float mapTo360(float angle) {
  return angle - 360 * (int(angle) / 360 + (angle < 0)) * ((angle >= 0) - (angle < 0));
}


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
aivision windshield = aivision(PORT9, aivision::ALL_AIOBJS);

struct aiVisionObjectWrapper {
    bool exists;
    aivision::object object;
};


class aivisionUtil{
  public:
  //aivision properties
    float aiXFov;
    float aiYFov;

    int xRes;
    int yRes; 

    float camHeight;

    float xResFovRatio;
    float yResFovRatio;

    float distance;//position from center of robot


  aivisionUtil (float aiXFov, float aiYFov, int xRes, int yRes, float camHeight){
    this->aiXFov = aiXFov; 
    this->aiYFov = aiYFov;
    this->xRes = xRes;
    this->yRes = yRes;
    this->camHeight = camHeight;
    


    xResFovRatio = aiXFov / xRes;
    yResFovRatio = aiYFov / yRes;

  }

  aiVisionObjectWrapper getClosest(aivision::objdesc target){
    aiVisionObjectWrapper returner;
    windshield.takeSnapshot(target, 8);

      if(windshield.objectCount == 0){
        returner.exists = false; 
        returner.object = aivision::object();
      } else if (windshield.objectCount != 0){
        returner.exists = true;
        returner.object = windshield.objects[0];
      }
    printf("objects: %i \n", windshield.objectCount);
    return returner; 
  }
  bool aiCenter(aivision::objdesc target){
    aiVisionObjectWrapper close = getClosest(target);
    
    double trueX = close.object.centerX - (xRes/2);
    
    double theta = trueX/((xRes * aiXFov)/2);

    printf("objects angle from: %d \n", theta);

    if(close.exists == true){

      if(close.object.centerX != 0){
        //enter theta
        turning.turnTo(theta);

        return true;
      } else {

        return true;
      }

    } else {
      return false; 
    }

  } 
  bool aiDistance(aivision::objdesc target){
    aiVisionObjectWrapper close = getClosest(target);


    bool truing = aiCenter(target);
    
    if(truing == false){
      return false;
    } else {
      int objWidth = close.object.width;

      float alphaOne = objWidth/(xRes * aiXFov);
      float zHigh = (objWidth/2)/(tan(alphaOne/2));

      float zTrue = sqrt(pow(zHigh, 2) - pow(camHeight, 2));

      moving.moveTo(zTrue);

      return true;
    }

  } 
};

aivisionUtil windshieldUtil = aivisionUtil(74, 63, 320, 240, 10.5);

//clamp initializations
bool doinker = false;
bool Adown = false;
pneumatics mogoClamp = pneumatics(Brain.ThreeWirePort.A);
pneumatics doink = pneumatics(Brain.ThreeWirePort.B);

class mmtoinches{
  Public:
static float inches(float m){
float x = m/25.4;
return x;
}
}

class unjammer{
  
  public:

  static void unjam(){
    int jamVar = 100;
    int lowIntHighRange = lIntMotor.position(rotationUnits::deg) + jamVar;
    int lowIntLowRange = lIntMotor.position(rotationUnits::deg) - jamVar;
    int hIntVelocity = hIntMotor.position(rotationUnits::deg);

    if(lowIntHighRange < hIntVelocity || lowIntLowRange > hIntVelocity){
      intake.spinFor(directionType::rev, 500, timeUnits::msec);
      intake.resetPosition();
    }
  }
};
//methods
  //intake methods
void intakeSpinFor(){
  intake.resetPosition();
  //unjammer::unjam();
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

bool releaser = false;

//clamp methods
class autoClamper{

  public:

    static void stopAutoClamping(){
      mogoClamp.set(true);
    }
  
  static void startAutoClamping(){
    float dist = clamper.objectDistance(distanceUnits::mm);

    if(Controller.ButtonR2.pressing() == true){
      stopAutoClamping();
    } else {
      if(dist <= 25){
        mogoClamp.set(false);
      } else {
        mogoClamp.set(true);
      }
    }
  }

};

//high steaks meathods
class highStakes{
  public:

static void align(){
float length = stakesSensor.objectDistance(distanceUnits::mm);
float driveDis = 0
driveDis = inches(lenght-x)
moving.moveto(driveDis);
}
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
  drive.resetPosition();
  drive.setStopping(brakeType::brake);
  intake.resetPosition();
  inert.calibrate();

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

  inert.calibrate();

  windshield.modelDetection(true);
  intake.setVelocity(100,velocityUnits::pct);
  drive.resetPosition();
  

  int slotter = 2;
    //EXPERIEMENTAL AUTON WITH AI-BASED PID
    if(slotter == 0){

    } else if(slotter == 1){ //four ring side master auton
      //drive towards goal, then slowly back into it
      autoClamper::startAutoClamping;
      moving.moveTo(-22);
      moving.moveTo(-6);
      moving.moveTo(-6);
      //clamp goal and wait .25 sec then move back
      mogoClamp.set(false);
      wait(250, timeUnits::msec);
      moving.moveTo(3);
      //put preload onto goal
      intake.spin(directionType::fwd);
      //turn to two ring stack
      wait(750, msec);
      turning.turnTo(-90);
      wait(750, timeUnits::msec);
      //intake bottom of two ring stack
      moving.moveTo(18);
      moving.moveTo(11);
      wait(500, timeUnits::msec);
      moving.moveTo(5);
      //DONE FINISHED


      wait(1000, timeUnits::msec);
      //move back and turn towards four rings
      moving.moveTo(-8);
      turning.turnTo(-170);
      //turn towards left ring
      //intake left ring
      moving.moveTo(15);
      wait(500, timeUnits::msec);
      moving.moveTo(-18);
      turning.turnTo(-180);

      //turn towards right ring
      //turning.turnTo(-177.5);
      //intake right ring
      moving.moveTo(20);
      moving.moveTo(-20);
      //turn around
      intake.stop();

    } else if(slotter == 2){
      moving.moveTo(-22);
      moving.moveTo(-6);
      moving.moveTo(-6);
      //clamp goal and wait .25 sec then move back
      mogoClamp.set(false);
      wait(250, timeUnits::msec);
      moving.moveTo(3);
      //put preload onto goal
      intake.spin(directionType::fwd);
      //turn to two ring stack
      wait(250, msec);
      turning.turnTo(-90);
      mogoClamp.set(true);
      wait(750, timeUnits::msec);
      //intake bottom of two ring stack
      moving.moveTo(18);
      moving.moveTo(11);
      wait(125, timeUnits::msec);
      intake.stop();
      moving.moveTo(5);
      moving.moveTo(-8);
      turning.turnTo(0);
      moving.moveTo(-17);
      moving.moveTo(-6);
      mogoClamp.set(false);
      moving.moveTo(6);
      intake.spin(directionType::fwd);

    } else if(slotter == 3){
      intake.spinFor(directionType::fwd, 1, timeUnits::sec);
      moving.moveTo(12);
      turning.turnTo(90);
      moving.moveTo(-12);
      moving.moveTo(-4);
      mogoClamp.set(false);
      moving.moveTo(-8);
      turning.turnTo(0);
      wait(250, timeUnits::msec);
      intake.spin(directionType::fwd);
      moving.moveTo(22);
      turning.turnTo(-90);
      moving.moveTo(30);

    } else if (slotter == 4){
      //drive towards goal, then slowly back into it
      autoClamper::startAutoClamping;
      moving.moveTo(-22);
      moving.moveTo(-6);
      moving.moveTo(-6);
      //clamp goal and wait .25 sec then move back
      mogoClamp.set(false);
      wait(250, timeUnits::msec);
      moving.moveTo(3);
      //put preload onto goal
      intake.spin(directionType::fwd);
      //turn to two ring stack
      wait(750, msec);
      turning.turnTo(-90);
      wait(750, timeUnits::msec);
      //intake bottom of two ring stack
      moving.moveTo(18);
      moving.moveTo(11);
      wait(500, timeUnits::msec);
      moving.moveTo(5);
      //DONE FINISHED

      mogoClamp.set(true);
      intake.stop();
      turning.turnTo(135);
      moving.moveTo(24);
      turning.turnTo(90);
      moving.moveTo(36);
      //win point two ring
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
    leftDrive.spin(directionType::fwd, (Controller.Axis3.value() + (Controller.Axis1.value()) ) * driveChanger, velocityUnits::pct);
    rightDrive.spin(directionType::fwd, (Controller.Axis3.value() - (Controller.Axis1.value()) ) * driveChanger, velocityUnits::pct);
    
  //intake code 
    //intake press
    Controller.ButtonL1.pressed(intakeSpinFor);
    Controller.ButtonL1.released(intakeStop);
    //outtake press
    Controller.ButtonL2.pressed(intakeSpinAga); // used to be DOWN
    Controller.ButtonL2.released(intakeStop); // used to be DOWN
    //clamp code
    //used to be L2
    autoClamper::startAutoClamping();
    //doink code
    Controller.ButtonR1.pressed(doinked);
    Controller.ButtonR1.released(undoink);
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
