/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       demetrieromero                                            */
/*    Created:      9/19/2024, 1:08:29 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include "math.h"


using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here
brain Brain;
controller Controller = controller();

//drivetrain initializations
motor fRMotor = motor(PORT19, ratio6_1, false);//front right drive motor
motor mRMotor = motor(PORT18, ratio6_1, false);//top right motor
motor rRMotor = motor(PORT17, ratio6_1, true);//rear right drive motor

motor fLMotor = motor(PORT14, ratio6_1, false);//front left drive motor
motor mLMotor = motor(PORT13, ratio6_1, true);//top left motor
motor rLMotor = motor(PORT12, ratio6_1, true);//rear left drive motor

inertial inert = inertial(PORT2);

motor_group leftDrive = motor_group(fLMotor, rLMotor, mLMotor);
motor_group rightDrive = motor_group(fRMotor, rRMotor, mRMotor);
motor_group drive = motor_group(fLMotor, rLMotor, mLMotor, fRMotor, rRMotor, mRMotor);

motor highL = motor(PORT20, ratio18_1, true); 
motor highR = motor(PORT11, ratio18_1, false);
motor_group ladyB = motor_group(highL, highR);


float driveChanger = 1.0;

float turnChanger(float x){
  return (1/10000) * (pow(x, 3));
}

//smartdrive blackjack = smartdrive(leftDrive, rightDrive, inert, 12.57, 10.625, 9.5, inches, 2);

//intake intializations
motor intake = motor(PORT21, ratio6_1, true);


optical colorSort = optical(PORT3);
optical clamper = optical(PORT10);

double iTD(double i){
  return i/((2.75*M_PI)/360);
}

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

   double initialHeading = 0.0;
   double currentHeading = 0.0;
   double hErrorL = 0.0;
   double hErrorR = 0.0;
   double rOutput = 0.0;
   double lOutput = 0.0;

   double p0 = 0.0;
   double p1 = 0.0;
   double p2 = 0.0;
   double p3 = 0.0;

   double leadIn = 0.5;

   double leadOut = 0.5;

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
      if(settle>80){
        error=0;
        leftDrive.stop();
        rightDrive.stop();
      }
      wait(50,msec);
    }
  }
  void moveTo(double target){
    drive.resetPosition();

    int settle = 0;
    motorEncoderLeft=fLMotor.position(degrees)*(2.75*M_PI)/360;
    motorEncoderRight=fRMotor.position(degrees)*(2.75*M_PI)/360;

    initialHeading = inert.rotation();
    currentHeading = inert.rotation();

    hErrorL = -1 * (initialHeading - currentHeading);
    hErrorR = initialHeading - currentHeading;

    errorL=target-motorEncoderLeft + hErrorL;
    errorR=target-motorEncoderRight + hErrorR;

    error=(errorL+errorR)/2;
    while(fabs(error)>0.1){
      motorEncoderLeft=fLMotor.position(degrees)*(2.75*M_PI)/360;
      motorEncoderRight=fRMotor.position(degrees)*(2.75*M_PI)/360;

      currentHeading = inert.rotation();

      // hErrorL = -2 * (initialHeading - currentHeading);
      // hErrorR = 2 * (initialHeading - currentHeading);
      hErrorL = 0;
      hErrorR = 0;

      errorL=target-motorEncoderLeft;
      errorR=target-motorEncoderRight;

      error=(errorL+errorR)/2;
      integral=integral+error;
      // if(error==0 range){
      //   integral=0;
      // } implement fastest route
      derivative=error-prevError;
      prevError=error;
      output=error*kP+integral*kI+derivative*kD * .8;
      lOutput=error*kP+integral*kI+derivative*kD;

      // rOutput = errorR*kP+integral*kI+derivative*kD;
      // lOutput = errorL*kP+integral*kI+derivative*kD;
      // leftDrive.spin(forward, lOutput, volt);
      // rightDrive.spin(forward, rOutput, volt);
      leftDrive.spin(forward, lOutput, volt);
      rightDrive.spin(forward, output, volt);
      printf("moveto: %f \n",error);
      // printf("left: %f \n", hErrorL);
      // printf("right: %f \n", hErrorR);
      if(error<1 && error>(-1)){
        settle++;
      }
      if(settle>300){
        error=0;
        leftDrive.stop();
        rightDrive.stop();
        leftDrive.resetPosition();
        rightDrive.resetPosition();
      }
            wait(10,msec);
    }

  //   p3 = target;
  //   p1 = target * leadIn;
  //   p2 = target - (target * leadOut);


  //   for (double t=0;t<1;t+=(1.0/30)){

	// //println t
	//     double x = pow((1.0-t),3) * p0 +3 * pow((1-t),2) * t * p1+3 * (1-t) * pow(t,2) * p2 + pow(t,3) * p3;

  //     drive.spinToPosition(x, vex::rotationUnits::deg, 100, vex::velocityUnits::pct);

  //     wait(10, msec);

  //   }
    leftDrive.resetPosition();
    rightDrive.resetPosition();
  }
};

PID turning = PID(0.4, 0.0, 0.65);
PID moving = PID(0.4, 0.0, 0.65);

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

bool releaser = false;

//clamp methods
class autoClamper{

  public:

    static void stopAutoClamping(){
      mogoClamp.set(false);
    }
  
  static void startAutoClamping(){
    double hue = clamper.hue();
    bool near = clamper.isNearObject();

    if(Controller.ButtonR2.pressing() == true){
      stopAutoClamping();
    } else {
      if(hue > 50 && hue < 68 && near){
        mogoClamp.set(true);
        wait(50, timeUnits::msec);
      } else {
        mogoClamp.set(false);
      }
    }
  }

};


  //doink methods
void ladyStop(){
  ladyB.stop();
}
void ladyUp(){
  ladyB.spinToPosition(-50, rotationUnits::deg, 30, velocityUnits::pct);
  ladyStop();
}
void ladyDown(){
  ladyB.spinToPosition(-200, rotationUnits::deg, 30, velocityUnits::pct);
}
void ladyQuit(){
  ladyB.setStopping(brakeType::coast);
  ladyB.spinTo(0, rotationUnits::deg, 2.5, velocityUnits::pct);
  ladyB.stop();
  ladyB.setStopping(brakeType::hold);
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
  intake.setVelocity(100,velocityUnits::pct);
  mogoClamp.set(false);
  drive.resetPosition();
  drive.setStopping(brakeType::brake);
  intake.resetPosition();

  inert.calibrate();

  ladyB.setStopping(brakeType::hold);
  clamper.setLight(ledState::on);
  clamper.setLightPower(100, percent);
  colorSort.setLight(ledState::on);
  colorSort.setLightPower(100, percent);

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
  drive.resetPosition();
  int slotter = 10;
    //EXPERIEMENTAL AUTON WITH AI-BASED PID
    if(slotter == 0){
  
      intake.spin(directionType::fwd);
      wait(500, msec);
      moving.moveTo(11);
      turning.turnTo(90);
      moving.moveTo(-8);
      moving.moveTo(-9);
      mogoClamp.set(true);
      wait(50, msec);
      turning.turnTo(-90);
      intake.spin(directionType::fwd);
      moving.moveTo(8);
      moving.moveTo(8);
      moving.moveTo(8);
      moving.moveTo(8);
      moving.moveTo(12);
      wait(50, msec);


    } else if(slotter == 1){ //four ring side master auton
      moving.moveTo(-11);
      moving.moveTo(-11);
      moving.moveTo(-4);
      mogoClamp.set(true);
      moving.moveTo(-8);
      //clamp goal and wait .25 sec then move back
      //put preload onto goal
      intake.spin(directionType::fwd);
      wait(100, msec);
      //turn to two ring stack
      turning.turnTo(-90);
      wait(25, msec);
      //intake bottom of two ring stack
      moving.moveTo(16);
      moving.moveTo(11);
      wait(200, msec);
      moving.moveTo(-7);
      turning.turnTo(-180);
      moving.moveTo(13);
      wait(250, msec);
      turning.turnTo(-160);
      moving.moveTo(-13);
    } else if(slotter == 10){

      moving.moveTo(-24);

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
    Controller.ButtonR1.pressed(ladyUp);
    Controller.ButtonY.pressed(ladyDown);
    Controller.ButtonB.pressed(ladyQuit);
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
