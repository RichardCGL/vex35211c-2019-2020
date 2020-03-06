#include "vex.h"
#include "robot-config.h"
#include "constants.h"
#include "controlLib.h"


/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the cortex has been powered on and    */ 
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void stopBot(void){
    motorGroupStop(wheelF,brakeType::hold);
    motorGroupStop(wheelB,brakeType::hold);
}


void statsMonitor(){
    Brain.Screen.setOrigin(1,1);
    while(true){
        for(int i=0;i<8;i++){
            Brain.Screen.printAt(1,20*i,true,"Motor %e Temperature: %e",i+1,
                               allMotors[i].temperature(percentUnits::pct));
        }
        task::sleep(timeDelay*4);
    }
}

void pre_auton(void) {
    wheelLF.setStopping(brakeType::brake);
    wheelLB.setStopping(brakeType::brake);
    wheelRF.setStopping(brakeType::brake);
    wheelRB.setStopping(brakeType::brake);
    thread statsMoitorTread(statsMonitor);
}

void autonomous(void);

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/
void driveCtrl(void){
    wheelLF.setStopping(brakeType::coast);
    wheelLB.setStopping(brakeType::coast);
    wheelRF.setStopping(brakeType::coast);
    wheelRB.setStopping(brakeType::coast);
    int stopMs = 0;
    while(true){
        
        double wheelCtlX = Controller1.Axis4.value()*0.5;
        double wheelCtlY = Controller1.Axis3.value();
        double theata = 0;
        
        if (wheelCtlY != 0){
            theata = fabs(atan(wheelCtlX/wheelCtlY))*180/M_PI; //get angle relative to to Y-axis
        }else{
            theata = 90;
        }
        
        if(theata < thresholdDeg){
            wheelCtlY *= CTR_TO_RPM;
            wheelCtlX *= CTR_TO_RPM_LESS;
        }else{
            wheelCtlY *= CTR_TO_RPM_LESS;
            wheelCtlX *= CTR_TO_RPM_LESS;
        }

        double wheelRpmL = wheelCtlY+wheelCtlX;
        double wheelRpmR = wheelCtlY-wheelCtlX;

        if(Controller1.ButtonL1.pressing()){
            wheelRpmL = 0;
        }else if(Controller1.ButtonL2.pressing()){
            wheelRpmR = 0;
        }
        
        motorGroupSpin(wheelL,directionType::fwd,wheelRpmL,velocityUnits::rpm);
        motorGroupSpin(wheelR,directionType::fwd,wheelRpmR,velocityUnits::rpm);
        
        //Coast when below threshold
        if (fabs(wheelRpmL) <= threshold){
            motorGroupStop(wheelL,brakeType::coast);
        }
        if (fabs(wheelRpmR) <= threshold){
            motorGroupStop(wheelR,brakeType::coast);
        }
        
        //Auto Hold after 500ms
        if (wheelCtlX == 0 && wheelCtlY == 0){
            stopMs += timeDelay;
            if (stopMs >= 500){
                motorGroupStop(wheelL,brakeType::hold);
                motorGroupStop(wheelR,brakeType::hold);
            }
        }else{
            stopMs = 0;
        }
        task::sleep(timeDelay);
    }
}

void intakeCtrl(void){
    while(true){
        if(Controller1.ButtonR1.pressing()){
            motorGroupSpin(intake,directionType::rev,75,velocityUnits::pct);
        }else if(Controller1.ButtonR2.pressing()){
            motorGroupSpin(intake,directionType::fwd,75,velocityUnits::pct);
        }else{
            motorGroupStop(intake,brakeType::hold);
        }
        task::sleep(timeDelay);
    }
}

void riseCtrl(void){
    while(true){
        if(Controller1.ButtonX.pressing()){
            rise.spin(directionType::fwd, 70, velocityUnits::pct);
        }else if(Controller1.ButtonB.pressing()){
            rise.spin(directionType::rev, 70, velocityUnits::pct);
        }else{
            rise.stop(brakeType::hold);
        }

        if(Controller1.ButtonY.pressing()){
          rise.spin(directionType::fwd, 70, velocityUnits::pct);
        }
        task::sleep(timeDelay);
    }
}

void pushCtrl(void){
    int axisPosition = 0;
    while(true){
      axisPosition = Controller1.Axis2.position(percentUnits::pct);
        if(axisPosition > 10){
          if(axisPosition > 30){
            axisPosition = 30;
          }
          push.spin(directionType::fwd, axisPosition, velocityUnits::pct);
        }else if(axisPosition < -10){
          if(axisPosition < -60){
             axisPosition = -60 ;
          }
            push.spin(directionType::fwd, axisPosition, velocityUnits::pct);
        }else{
            push.stop(brakeType::hold);
        }

        if(Controller1.ButtonY.pressing()){
          push.spin(directionType::fwd, 20, velocityUnits::pct);
        }
        task::sleep(timeDelay);
    }
}

void usercontrol() {
    thread driveCtrlThread(driveCtrl);
    thread inTakeCtrlThread(intakeCtrl);
    thread riseCtrlThread(riseCtrl);
    thread pushCtrlThread(pushCtrl);

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
void autonomous(){

  //base_train.turnFor(double angle, rotationUnits units, double velocity, velocityUnits units_v);
  //base_train.driveFor(double distance, distanceUnits units, double velocity, velocityUnits units_v);

    push.spin(vex::directionType::rev, 50, vex::velocityUnits::pct);
    intakeMotorL.spin(vex::directionType::fwd, 80, vex::velocityUnits::pct);
    intakeMotorR.spin(vex::directionType::fwd, 80, vex::velocityUnits::pct);
    vex::task::sleep(200);
    rise.spin(vex::directionType::fwd, 10, vex::velocityUnits::pct);
    vex::task::sleep(100);
    rise.spin(vex::directionType::rev, 10, vex::velocityUnits::pct);
    vex::task::sleep(100);
    rise.stop(vex::brakeType::hold);
    push.stop(vex::brakeType::hold);

    vex::task::sleep(800);

  base_train.driveFor(70, distanceUnits::in, 100, velocityUnits::pct);
  
  stopBot();
  vex::task::sleep(400);

  intakeMotorL.stop(vex::brakeType::hold);
  intakeMotorR.stop(vex::brakeType::hold);

  base_train.driveFor(-50, distanceUnits::in, 80, velocityUnits::pct);

  stopBot();
  vex::task::sleep(400);

  base_train.turnFor(-140, deg, 70, velocityUnits::pct);

  stopBot();
  vex::task::sleep(400);

  base_train.driveFor(30, distanceUnits::in, 60, velocityUnits::pct);

  stopBot();
  vex::task::sleep(400);

  push.spin(vex::directionType::fwd, 90, vex::velocityUnits::pct);
  vex::task::sleep(1000);
  push.stop(vex::brakeType::hold);

  base_train.driveFor(-15, distanceUnits::in, 60, velocityUnits::pct);

}

int main() {
    // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  pre_auton();
  Competition.autonomous( autonomous );
  Competition.drivercontrol( usercontrol ); 
}
