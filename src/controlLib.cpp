#include "vex.h"
/*
void pidMove(int goal, vex::motor theMotor){

  int previousError = 0;
  int totalError = 0;

  float kP = 0.054;
  float kI = 0.043;
  float kD = 0.42;

  int error = theMotor.position(vex::deg) - goal;
  totalError += error;
  int derivative = error - previousError;

  int motorVelocity = (error * kP + totalError * kI + derivative * kD); //Add values up
  theMotor.spin() = motorVelocity; //Apply values to motor
//Since code is read from top to bottom, this will be read, 
//then 20 miliseconds will pass before the next loop causing this value
//To become a value 20 miliseconds ago
  previousError = error; 
  vex::task::sleep(20);
}
*/
void motorGroupSpin(vex::motor_group motorGroup[], vex::directionType dirType,double velval, vex::velocityUnits velType){
    motorGroup[0].spin(dirType,velval,velType);
    motorGroup[1].spin(dirType,velval,velType);
}

void motorGroupStop(vex::motor_group motorGroup[], vex::brakeType brkType){
    motorGroup[0].stop(brkType);
    motorGroup[1].stop(brkType);
}

void goToTime(int left, int right){     
        wheelLF.spin(vex::directionType::fwd,left,vex::velocityUnits::pct);
        wheelRF.spin(vex::directionType::fwd,right,vex::velocityUnits::pct);
        wheelLB.spin(vex::directionType::fwd,left,vex::velocityUnits::pct);
        wheelRB.spin(vex::directionType::fwd,right,vex::velocityUnits::pct);
}

void asyncMoveDistance(vex::motor_group motorGroup[], float distance, int * status){
  


}
