#include "vex.h"
using namespace vex;
#include "robot-config.h"
#include "controlLib.h"


vex::competition Competition;

vex::motor_group wheelF[2] = {wheelLF,wheelRF};
vex::motor_group wheelB[2] = {wheelLB,wheelRB};

vex::motor_group wheelL[2] = {wheelLF,wheelLB};
vex::motor_group wheelR[2] = {wheelRF,wheelRB};

vex::motor_group intake[2] = {intakeMotorL,intakeMotorR};

vex::motor_group allMotors[8] = {wheelLF,   wheelLB,    wheelRF,    wheelRB,
                                //1       2           3           4
                                rise,   push,       intakeMotorL,intakeMotorR};
                                //5       6           7           8


//Tune here


/*---------------------------------------------------------------------------*/
/*                           Self-Defined Functions                          */
/*---------------------------------------------------------------------------*/



void stopBot(){
    motorGroupStop(wheelF,vex::brakeType::hold);
    motorGroupStop(wheelB,vex::brakeType::hold);
}

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the cortex has been powered on and    */ 
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/


void statsMonitor(){
    Brain.Screen.setOrigin(1,1);
    while(true){
        for(int i=0;i<8;i++){
            Brain.Screen.printAt(1,20*i,true,"Motor %e Temperature: %e",i+1,
                               allMotors[i].temperature(vex::percentUnits::pct));
        }
        vex::task::sleep(timeDelay*4);
    }
}

void pre_auton(void) {
    wheelLF.setStopping(vex::brakeType::brake);
    wheelLB.setStopping(vex::brakeType::brake);
    wheelRF.setStopping(vex::brakeType::brake);
    wheelRB.setStopping(vex::brakeType::brake);
    vex::thread statsMoitorTread(statsMonitor);
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
    wheelLF.setStopping(vex::brakeType::coast);
    wheelLB.setStopping(vex::brakeType::coast);
    wheelRF.setStopping(vex::brakeType::coast);
    wheelRB.setStopping(vex::brakeType::coast);
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
        
        motorGroupSpin(wheelL,vex::directionType::fwd,wheelRpmL,vex::velocityUnits::rpm);
        motorGroupSpin(wheelR,vex::directionType::fwd,wheelRpmR,vex::velocityUnits::rpm);
        
        //Coast when below threshold
        if (fabs(wheelRpmL) <= threshold){
            motorGroupStop(wheelL,vex::brakeType::coast);
        }
        if (fabs(wheelRpmR) <= threshold){
            motorGroupStop(wheelR,vex::brakeType::coast);
        }
        
        //Auto Hold after 500ms
        if (wheelCtlX == 0 && wheelCtlY == 0){
            stopMs += timeDelay;
            if (stopMs >= 500){
                motorGroupStop(wheelL,vex::brakeType::hold);
                motorGroupStop(wheelR,vex::brakeType::hold);
            }
        }else{
            stopMs = 0;
        }
        vex::task::sleep(timeDelay);
    }
}

void intakeCtrl(void){
    while(true){
        if(Controller1.ButtonR1.pressing()){
            motorGroupSpin(intake,vex::directionType::rev,75,vex::velocityUnits::pct);
        }else if(Controller1.ButtonR2.pressing()){
            motorGroupSpin(intake,vex::directionType::fwd,75,vex::velocityUnits::pct);
        }else{
            motorGroupStop(intake,vex::brakeType::hold);
        }
        vex::task::sleep(timeDelay);
    }
}

void riseCtrl(void){
    while(true){
        if(Controller1.ButtonX.pressing()){
            rise.spin(vex::directionType::fwd, 70, vex::velocityUnits::pct);
        }else if(Controller1.ButtonB.pressing()){
            rise.spin(vex::directionType::rev, 70, vex::velocityUnits::pct);
        }else{
            rise.stop(vex::brakeType::hold);
        }

        if(Controller1.ButtonY.pressing()){
          rise.spin(vex::directionType::fwd, 70, vex::velocityUnits::pct);
        }
        vex::task::sleep(timeDelay);
    }
}

void pushCtrl(void){
    int axisPosition = 0;
    while(true){
      axisPosition = Controller1.Axis2.position(vex::percentUnits::pct);
        if(axisPosition > 10){
          if(axisPosition > 30){
            axisPosition = 30;
          }
          push.spin(vex::directionType::fwd, axisPosition, vex::velocityUnits::pct);
        }else if(axisPosition < -10){
          if(axisPosition < -60){
             axisPosition = -60 ;
          }
            push.spin(vex::directionType::fwd, axisPosition, vex::velocityUnits::pct);
        }else{
            push.stop(vex::brakeType::hold);
        }

        if(Controller1.ButtonY.pressing()){
          push.spin(vex::directionType::fwd, 20, vex::velocityUnits::pct);
        }
        vex::task::sleep(timeDelay);
    }
}

void usercontrol() {
    vex::thread driveCtrlThread(driveCtrl);
    vex::thread inTakeCtrlThread(intakeCtrl);
    vex::thread riseCtrlThread(riseCtrl);
    vex::thread pushCtrlThread(pushCtrl);

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

    //first row
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
    
    //second
    wheelLF.setVelocity(23, pct);
    wheelLB.setVelocity(23, pct);
    wheelRF.setVelocity(23, pct);
    wheelRB.setVelocity(23, pct);

    wheelLF.rotateFor(1290, vex::rotationUnits::deg, false);
    wheelRF.rotateFor(1290, vex::rotationUnits::deg, false);
    wheelRB.rotateFor(1290, vex::rotationUnits::deg, false);
    wheelLB.rotateFor(1290, vex::rotationUnits::deg, true);

    stopBot();
    vex::task::sleep(400);

    intakeMotorL.stop(vex::brakeType::hold);
    intakeMotorR.stop(vex::brakeType::hold);

    //turn 2
    // goToTime(-50, 50);
    // vex::task::sleep(800);
    wheelLF.setVelocity(-75, pct);
    wheelLB.setVelocity(-75, pct);
    wheelRF.setVelocity(-75, pct);
    wheelRB.setVelocity(-75, pct);

    wheelLF.rotateFor(-930, vex::rotationUnits::deg, false);
    wheelRF.rotateFor(-930, vex::rotationUnits::deg, false);
    wheelRB.rotateFor(-930, vex::rotationUnits::deg, false);
    wheelLB.rotateFor(-930, vex::rotationUnits::deg, true);

    stopBot();
    vex::task::sleep(100);

    //go back
    // goToTime(80, 80);
    // vex::task::sleep(1200);
    // stopBot();

    wheelLF.setVelocity(-50, pct);
    wheelLB.setVelocity(-50, pct);
    wheelRF.setVelocity(50, pct);
    wheelRB.setVelocity(50, pct);

    wheelLF.rotateFor(-413, vex::rotationUnits::deg, false);
    wheelRF.rotateFor(413, vex::rotationUnits::deg, false);
    wheelRB.rotateFor(413, vex::rotationUnits::deg, false);
    wheelLB.rotateFor(-413, vex::rotationUnits::deg, true);

    stopBot();
    vex::task::sleep(300);

    wheelLF.setVelocity(70, pct);
    wheelLB.setVelocity(70, pct);
    wheelRF.setVelocity(70, pct);
    wheelRB.setVelocity(70, pct);

    wheelLF.rotateFor(550, vex::rotationUnits::deg, false);
    wheelRF.rotateFor(550, vex::rotationUnits::deg, false);
    wheelRB.rotateFor(550, vex::rotationUnits::deg, false);
    wheelLB.rotateFor(550, vex::rotationUnits::deg, true);

    //put
    push.spin(vex::directionType::fwd, 30, vex::velocityUnits::pct);
    vex::task::sleep(400);
    push.stop();

    intakeMotorL.spin(vex::directionType::rev, 70, vex::velocityUnits::pct);
    intakeMotorR.spin(vex::directionType::rev, 70, vex::velocityUnits::pct);
    vex::task::sleep(300);
    intakeMotorL.stop(vex::brakeType::coast);
    intakeMotorR.stop(vex::brakeType::coast);

    push.spin(vex::directionType::fwd, 40, vex::velocityUnits::pct);
    vex::task::sleep(300);
    push.stop();

    intakeMotorL.spin(vex::directionType::rev, 70, vex::velocityUnits::pct);
    intakeMotorR.spin(vex::directionType::rev, 70, vex::velocityUnits::pct);
    vex::task::sleep(300);
    intakeMotorL.stop(vex::brakeType::coast);
    intakeMotorR.stop(vex::brakeType::coast);

    push.spin(vex::directionType::fwd, 40, vex::velocityUnits::pct);
    vex::task::sleep(300);
    push.stop();

    intakeMotorL.spin(vex::directionType::rev, 70, vex::velocityUnits::pct);
    intakeMotorR.spin(vex::directionType::rev, 70, vex::velocityUnits::pct);
    vex::task::sleep(300);
    intakeMotorL.stop(vex::brakeType::coast);
    intakeMotorR.stop(vex::brakeType::coast);

    push.spin(vex::directionType::fwd, 40, vex::velocityUnits::pct);
    vex::task::sleep(1000);
    push.stop();
    vex::task::sleep(500);

    goToTime(20, 20);

    vex::task::sleep(350);

    intakeMotorL.spin(vex::directionType::rev, 25, vex::velocityUnits::pct);
    intakeMotorR.spin(vex::directionType::rev, 25, vex::velocityUnits::pct);
    goToTime(-50, -50);
    vex::task::sleep(200);

    intakeMotorL.stop(vex::brakeType::coast);
    intakeMotorR.stop(vex::brakeType::coast);
    vex::task::sleep(1000);
    stopBot();

}

int main() {
    // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();


  pre_auton();
  Competition.autonomous( autonomous );
  Competition.drivercontrol( usercontrol ); 
}
