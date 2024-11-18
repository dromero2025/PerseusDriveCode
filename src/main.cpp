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

void xyloPrint(){
  Brain.Screen.drawImageFromFile("xylo1 Small.png", 0, 0);
}
void xylo1Print(){
  //Brain.Screen.clearScreen();
  Brain.Screen.drawImageFromFile("xylo2.png", 0, 0);
}
void kamalaPrint(){
  //Brain.Screen.clearScreen();
  Brain.Screen.drawImageFromFile("kamala Small.png", 0, 0);
}

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

float avgTemp(){
  while(true){
    float rightDriveTemp = (fRMotor.temperature(temperatureUnits::celsius)+rRMotor.temperature(temperatureUnits::celsius)+topRMotor.temperature(temperatureUnits::celsius))/3.0;
    float leftDriveTemp = (fLMotor.temperature(temperatureUnits::celsius)+rLMotor.temperature(temperatureUnits::celsius)+topLMotor.temperature(temperatureUnits::celsius))/3.0;
    float driveTemp = (rightDriveTemp + leftDriveTemp)/2.0;
    float intakeTemp = (lIntMotor.temperature(temperatureUnits::celsius)+hIntMotor.temperature(temperatureUnits::celsius))/2.0;
    Brain.Screen.setFont(monoL);
    Brain.Screen.print("right drive heat: ");
    Brain.Screen.print(rightDriveTemp);
    Brain.Screen.newLine();
    Brain.Screen.print("left drive heat: ");
    Brain.Screen.print(leftDriveTemp);
    Brain.Screen.newLine();
    Brain.Screen.print("drive heat: ");
    Brain.Screen.print(driveTemp);
    Brain.Screen.newLine();
    Brain.Screen.print("intake heat: ");
    Brain.Screen.print(intakeTemp);
    Brain.Screen.setCursor(1,1);
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

//PID
class PID{
 public:
   double inertVal = 0.0;
   double motorEncoderLeft = 0.0;
   double motorEncoderRight = 0.0;
   double error = 0.0;
   double errorL = 0.0;
   double errorR = 0.0;
   double kP = 0.0;
   double kI = 0.0;
   double kD = 0.0;
   double integral = 0.0;
   double derivative = 0.0;
   double prevError = 0.0;
   double output = 0.0;

   PID(){ //basic constructror
   }

   PID(double kP, double kI, double kD){ //sets up k values
     this->kP = kP;
     this->kI = kI;
     this->kD = kD;
   }
    void turnTo(double target){ //turn to n degrees
     int settle = 0;
     inertVal = inert.rotation(rotationUnits::deg);
     error = target - inertVal;
     while(fabs(error) > 0.1){
       inertVal = inert.rotation(rotationUnits::deg);
       error = target - inertVal;
       integral = integral + error;
       // if(error==0 range){
       //   integral=0;
       // } implement fastest route
       derivative = error - prevError;
       prevError = error;
       output = error * kP + integral * kI + derivative * kD;
       leftDrive.spin(directionType::fwd, output, voltageUnits::volt);
       rightDrive.spin(directionType::rev, output, voltageUnits::volt);
       printf("turnto %f \n",error);
       if(error < 5 && error > (-5)){
         settle++;
       }
       if(settle > 50){
         error = 0;
         leftDrive.stop();
         rightDrive.stop();
       }
       wait(10,msec);
     }
   }
   void moveTo(double target){ //move to n dis
     int settle = 0;
     motorEncoderLeft = fLMotor.position(rotationUnits::deg) * (5.417 * M_PI) / 360;
     motorEncoderRight = fRMotor.position(rotationUnits::deg) * (5.417 * M_PI) / 360;
     errorL = target - motorEncoderLeft;
     errorR = target - motorEncoderRight;
     error = (errorL+errorR) / 2;
     while(fabs(error) > 0.1){
       motorEncoderLeft = fLMotor.position(rotationUnits::deg) * (5.417 * M_PI) / 360;
       motorEncoderRight = fRMotor.position(rotationUnits::deg) * (5.417 * M_PI) / 360;
       errorL = target - motorEncoderLeft;
       errorR = target - motorEncoderRight;
       error = (errorL + errorR) / 2;
       integral = integral + error;
       // if(error==0 range){
       //   integral=0;
       // } implement fastest route
       derivative = error - prevError;
       prevError = error;
       output = error * kP + integral * kI + derivative * kD;
       leftDrive.spin(directionType::fwd, output, voltageUnits::volt);
       rightDrive.spin(directionType::fwd, output, voltageUnits::volt);
       printf("moveto: %f \n",error);
       if(error < 5 && error > (-5)){
         settle++;
       }
       if(settle > 50){
         error = 0;
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
PID turning = PID(0.3, 0.0, 0.0);
PID moving = PID(0.4, 0.0, 0.0);

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
  intake.setVelocity(100,velocityUnits::pct);


  //RIGHT SIDE
  //drive towards mobile goal
  drive.spinFor(directionType::rev, 900, rotationUnits::deg, 50, velocityUnits::pct);
  drive.stop();
  //clamp goal
  mogoClamp.set(false);
  //load preload onto goal
  intake.spinFor(directionType::fwd, 1000, rotationUnits::deg, 100, velocityUnits::pct);
  drive.resetPosition();
  //turn right
  leftDrive.spinFor(directionType::rev, 225, rotationUnits::deg, 25, velocityUnits::pct);
  rightDrive.spinFor(directionType::rev, -400, rotationUnits::deg, 25, velocityUnits::pct);
  drive.resetPosition();
  //intake ring
  intake.spin(directionType::fwd);
  drive.spinFor(directionType::fwd, 1500, rotationUnits::deg, 100, velocityUnits::pct);
  wait(375, timeUnits::msec);
  //drive back towards tower
  intake.stop();
  drive.spinFor(directionType::rev, 1650, rotationUnits::deg, 100, velocityUnits::pct);
  drive.resetPosition();
  wait(1000, timeUnits::msec);
  intake.stop();

  //RIGHT SIDE W/Intertial
  //move towards goal
  moving.moveTo(24.0);
  //clamp
  mogoClamp.set(false);
  //intake preload onto goal, keep intake running
  intake.spin(directionType::fwd);
  //turn towards ring stack
  turning.turnTo(90.0);
  //move into rings
  moving.moveTo(24);

  //LEFT SIDE
  //drive towards mobile goal
  drive.spinFor(directionType::rev, 900, rotationUnits::deg, 50, velocityUnits::pct);
  drive.stop();
  //clamp goal
  mogoClamp.set(false);
  //load preload onto goal
  intake.spinFor(directionType::fwd, 1000, rotationUnits::deg, 100, velocityUnits::pct);
  drive.resetPosition();
  //turn left
  rightDrive.spinFor(directionType::rev, 225, rotationUnits::deg, 25, velocityUnits::pct);
  leftDrive.spinFor(directionType::rev, -400, rotationUnits::deg, 25, velocityUnits::pct);
  drive.resetPosition();
  //intake ring
  intake.spin(directionType::fwd);
  drive.spinFor(directionType::fwd, 1500, rotationUnits::deg, 100, velocityUnits::pct);
  wait(375, timeUnits::msec);
  //drive back towards tower
  intake.stop();
  drive.spinFor(directionType::rev, 1650, rotationUnits::deg, 100, velocityUnits::pct);
  drive.resetPosition();
  wait(1000, timeUnits::msec);
  intake.stop();
  
  //LEFT SIDE W/Intertial
  //move towards goal
  moving.moveTo(24.0);
  //clamp
  mogoClamp.set(false);
  //intake preload onto goal, keep intake running
  intake.spin(directionType::fwd);
  //turn towards ring stack
  turning.turnTo(-90.0);
  //move into rings
  moving.moveTo(24);


  //Theoretical Left Side With PID

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
