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
distance clampLeft = distance(PORT15);
distance clampRight = distance(PORT16);
//clamp initializations
bool doinker = false;
bool Adown = false;
pneumatics mogoClamp = pneumatics(Brain.ThreeWirePort.A);
pneumatics doink = pneumatics(Brain.ThreeWirePort.B);

rotation highStake = rotation(PORT9);

double iTD(double i){
  return i/((2.75*M_PI)/360);
}
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
bool stopClamp = false;
int clampDist = 65;
class autoClamper{
  public:
    
  

    static void stopAutoClamping(){
      stopClamp = true;
    }
    static void startAutoClamping(){
      stopClamp = false;
    }

  
  static task AutoClamping(){
    while(true){
      double lDist = clampLeft.objectDistance(distanceUnits::mm);
      double rDist = clampRight.objectDistance(distanceUnits::mm);
      double dist = (lDist + rDist)/2;
      double hue = clamper.hue();
      
      //printf("auto clamp intitated");
      printf("right distance: %f", rDist);
      printf(" left distance: %f \n", lDist);
      printf("hue: %f \n", hue);

      if(stopClamp == true){
        mogoClamp.set(false);
      } else if(stopClamp == false){
        if(dist <= clampDist && (lDist <= clampDist && rDist <= clampDist)){
          
            mogoClamp.set(true);
            printf("right distance: %f", rDist);
            printf(" left distance: %f \n", lDist);
            printf("hue: %f \n", hue);
            

          
        } 
      } else {
        mogoClamp.set(false);
      }
      wait(10, timeUnits::msec);
    }
    
    return 0;
  }
};
bool isBlue = false;
bool isRed = true;
int hue = colorSort.hue();
class colorSorter{
  
  public: 

  static void allowBlue(){
    isBlue = true;
  }
  static void stopAllowBlue(){
    isBlue = false;
  }
  static void allowRed(){
    isRed = true;
  }
  static void stopAllowRed(){
    isRed = false;
  }
  
  static void colorSorting(){
    //while(true){
      
      if(isBlue = false){
        if(colorSort.hue() <= 220 && colorSort.hue() >= 170){
          Brain.Screen.clearLine();
          Brain.Screen.print("get that blue ring outta here");
          wait(20, msec);
          intake.resetPosition();
          intake.spinToPosition(720, rotationUnits::deg, true);
          intake.setStopping(brakeType::brake);
          intake.stop();
          wait(2000, msec);
        }
      } else if(isRed = false){
        if(colorSort.hue() <= 15 && colorSort.hue() >= 8){
          Brain.Screen.clearLine();
          Brain.Screen.print("get that red ring outta here");
          wait(20, msec);
          intake.resetPosition();
          intake.spinToPosition(720, rotationUnits::deg, true);
          intake.setStopping(brakeType::brake);
          intake.stop();
          wait(2000, msec);
        }
      } else {
        Controller.ButtonL1.pressed(intakeSpinFor);
        Controller.ButtonL1.released(intakeStop);
        //outtake press
        Controller.ButtonL2.pressed(intakeSpinAga); // used to be DOWN
        Controller.ButtonL2.released(intakeStop); // used to be DOWN
      }

    //}
  }
};
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

   PID(){
   }
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
       if(error<10&&error>(-10)){
         settle++;
       }
       if(settle>50){
         error=0;
         leftDrive.stop();
         rightDrive.stop();
       }
       wait(10,msec);
     }
     
   }

   void moveTo(double target){
      
     int settle = 0;
     motorEncoderLeft=fLMotor.position(degrees)*(2.75*M_PI)/360;
     motorEncoderRight=fRMotor.position(degrees)*(2.75*M_PI)/360;
     errorL=target-motorEncoderLeft;
     errorR=target-motorEncoderRight;
     error=(errorL+errorR)/2;
     while(fabs(error)>0.1){
      
       motorEncoderLeft=fLMotor.position(degrees)*(2.75*M_PI)/360;
       motorEncoderRight=fRMotor.position(degrees)*(2.75*M_PI)/360;
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
       if(error<5&&error>(-5)){
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
PID turning = PID(0.17, 0.0, 0);
PID moving = PID(0.4, 0.0, 0);

  enum VisionSignatures {
    redRing,
    blueRing
  };

  // all object descriptions for game elements
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
    windshield.takeSnapshot(target, 2);

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
};

aivisionUtil windshieldUtil = aivisionUtil(74, 63, 320, 240, 10.5);

//methods
  //intake methods

bool releaser = false;

  //doink methods
bool loaded = false;
bool scoring = false;
int loadDeg = 320;
int scoreDeg = 260;
int highStakeError = 2;
void ladyUp(){
  if(loaded == true){
    if(scoring == true){
      ladyB.spinToPosition(-55, rotationUnits::deg, 15, velocityUnits::pct);
      ladyB.setStopping(brakeType::hold);
      ladyB.stop();
      Brain.Screen.clearLine();
      Brain.Screen.print("lady brown loaded");
      scoring = false;
    } else if (scoring == false){
      ladyB.spinToPosition(-180, rotationUnits::deg, 60, velocityUnits::pct);
      ladyB.setStopping(brakeType::hold);
      ladyB.stop();
      Brain.Screen.clearLine();
      Brain.Screen.print("lady brown scoring");
      scoring = true;
    }
  } else {
    ladyB.spinToPosition(-53, rotationUnits::deg, 10, velocityUnits::pct);
    ladyB.setStopping(brakeType::hold);
    ladyB.stop();
    Brain.Screen.clearLine();
    Brain.Screen.print("lady brown loaded");
    loaded = true;
    scoring = false;
  }
}
void ladyQuit(){
  ladyB.setStopping(brakeType::coast);
  
  ladyB.stop();
  wait(500, msec);
  ladyB.setStopping(brakeType::hold);
  Brain.Screen.clearLine();
  Brain.Screen.print("lady brown quit");
  loaded = false;
}

void doinked(){
  //mogoClamp.set(true);

  if(Adown == false){
    doink.set(doinker);
    doinker = !doinker;
  }
  
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
  intake.setVelocity(100,velocityUnits::pct);
  mogoClamp.set(false);
  drive.resetPosition();
  intake.resetPosition();
  ladyB.resetPosition();

  inert.calibrate();

  while(inert.isCalibrating()){
    wait(100,msec);
  }
  ladyB.setStopping(brakeType::hold);
  clamper.setLight(ledState::on);
  clamper.setLightPower(100, percent);
  colorSort.setLight(ledState::on);
  colorSort.setLightPower(50, percent);

  task controlTask(autoClamper::AutoClamping());
  //task controlTask(colorSorter::colorSorting());

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
  intake.resetPosition();
  while(true){
    //autoClamper::AutoClamping();
    //EXPERIEMENTAL AUTON WITH AI-BASED PID
    // if(slotter == 0){
      
      // //SKILLS CODE
      //   intake.spinToPosition(240, rotationUnits::deg, true);
      //   moving.moveTo(13);
      //   turning.turnTo(45);
      //   turning.turnTo(90);
      //   moving.moveTo(-20);
      //   wait(200, msec);
      //   moving.moveTo(-8);
      //   wait(300,msec);
      //   turning.turnTo(0);
      //   wait(100, msec);
      //   intake.spin(directionType::fwd);
      //   moving.moveTo(29);
      //   wait(400, msec);
      //   turning.turnTo(-45);
      //   turning.turnTo(-90);
      //   wait(300, msec);
      //   moving.moveTo(12);
      //   wait(600, msec);
      //   moving.moveTo(16);
      //   wait(200, msec);
      //   turning.turnTo(-135);
      //   turning.turnTo(-180);
      //   wait(100, msec);
      //   moving.moveTo(21); 
      //   moving.moveTo(21);  
      //   wait(500, msec);
      //   moving.moveTo(-20);
      //   wait(-200, msec);
      //   turning.turnTo(-145);
      //   wait(100, msec);
      //   moving.moveTo(24);
      //   wait(300, msec);
      //   //intake.stop();

      //   moving.moveTo(-4);
      //   turning.turnTo(-90);
        
      //   turning.turnTo(-45);
      //   turning.turnTo(0);
      //   turning.turnTo(45);
      //   moving.moveTo(-5);
      //   autoClamper::stopAutoClamping();

      //   //left side

      //   moving.moveTo(5);
      //   turning.turnTo(0);
      //   turning.turnTo(-45);
      //   turning.turnTo(-90);
      //   autoClamper::startAutoClamping();
      //   intake.resetPosition();
      //   intake.spin(directionType::fwd);

      //   moving.moveTo(-36); 
      //   //moving.moveTo(-18);

      //   turning.turnTo(-93);
      //   moving.moveTo(-34); 
      //   turning.turnTo(-95);
      //   //moving.moveTo(-16); 
      //   turning.turnTo(-99);
      //   moving.moveTo(-17); 
      //   wait(300, msec);
      //   //right side
      //   turning.turnTo(0);
      //   wait(100, msec);
      //   intake.spin(directionType::fwd);
      //   moving.moveTo(26);
      //   wait(400, msec);
      //   turning.turnTo(45);
      //   turning.turnTo(90);
      //   wait(300, msec);
      //   moving.moveTo(12);
      //   wait(600, msec);
      //   moving.moveTo(16);
      //   wait(200, msec);
      //   turning.turnTo(135);
      //   turning.turnTo(180);
      //   wait(100, msec);
      //   moving.moveTo(21); 
      //   moving.moveTo(21);  
      //   wait(500, msec);
      //   moving.moveTo(-20);
      //   wait(-200, msec);
      //   turning.turnTo(125);
      //   wait(100, msec);
      //   moving.moveTo(24);
      //   wait(300, msec);
      //   //intake.stop();

      //   moving.moveTo(-4);
      //   turning.turnTo(90);
        
      //   turning.turnTo(45);
      //   turning.turnTo(0);
      //   turning.turnTo(-45);
      //   moving.moveTo(-8);
      //   autoClamper::stopAutoClamping();




        

        

        
      
    // } else if(slotter == 1){ //four ring side master auton
    //THREE RING AUTON
      moving.moveTo(-13);
      moving.moveTo(-13);
      
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
      moving.moveTo(13);
      moving.moveTo(11);
      wait(200, msec);
      moving.moveTo(-5);
      turning.turnTo(-180);
      moving.moveTo(13);
      wait(250, msec);
      turning.turnTo(-160);
      moving.moveTo(-13);
 
    
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
    
    // colorSorter::allowRed();
    // colorSorter::stopAllowBlue();
    colorSorter::colorSorting();
  //drivetrain code
    leftDrive.spin(directionType::fwd, (Controller.Axis3.value() + (Controller.Axis1.value()) ) * driveChanger, velocityUnits::pct);
    rightDrive.spin(directionType::fwd, (Controller.Axis3.value() - (Controller.Axis1.value()) ) * driveChanger, velocityUnits::pct);
    
  //intake code 
    //intake press
    // Controller.ButtonL1.pressed(intakeSpinFor);
    // Controller.ButtonL1.released(intakeStop);
    // //outtake press
    // Controller.ButtonL2.pressed(intakeSpinAga); // used to be DOWN
    // Controller.ButtonL2.released(intakeStop); // used to be DOWN
    //clamp code
    //used to be L2
    //autoClamper::AutoClamping();
    if(Controller.ButtonR2.pressing()){
      autoClamper::stopAutoClamping();
    } else {
      autoClamper::startAutoClamping();
    }
    //doink code
    Controller.ButtonR1.pressed(ladyUp);
    Controller.ButtonY.pressed(ladyQuit);
    //Controller.ButtonB.pressed(ladyQuit);
    //avgTemp();
    Controller.ButtonA.pressed(doinked);
    Controller.ButtonA.released(undoink);
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
