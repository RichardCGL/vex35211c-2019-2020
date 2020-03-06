#include "vex.h"
#include "robot-config.h"
/*
void pidMove(int goal, motor theMotor){

  int previousError = 0;
  int totalError = 0;

  float kP = 0.054;
  float kI = 0.043;
  float kD = 0.42;

  int error = theMotor.position(deg) - goal;
  totalError += error;
  int derivative = error - previousError;

  int motorVelocity = (error * kP + totalError * kI + derivative * kD); //Add values up
  theMotor.spin() = motorVelocity; //Apply values to motor
//Since code is read from top to bottom, this will be read, 
//then 20 miliseconds will pass before the next loop causing this value
//To become a value 20 miliseconds ago
  previousError = error; 
  task::sleep(20);
}
*/
void motorGroupSpin(motor_group motorGroup[], directionType dirType,double velval, velocityUnits velType){
    motorGroup[0].spin(dirType,velval,velType);
    motorGroup[1].spin(dirType,velval,velType);
}

void motorGroupStop(motor_group motorGroup[], brakeType brkType){
    motorGroup[0].stop(brkType);
    motorGroup[1].stop(brkType);
}

void goToTime(int left, int right){     
        wheelLF.spin(directionType::fwd,left,velocityUnits::pct);
        wheelRF.spin(directionType::fwd,right,velocityUnits::pct);
        wheelLB.spin(directionType::fwd,left,velocityUnits::pct);
        wheelRB.spin(directionType::fwd,right,velocityUnits::pct);
}

/*
void syncMoveDistance(motor_group[],int motor_count,  float distance){

}

void asyncMoveDistance(motor_group motorGroup[], int motor_count, float distance, int * status){
  syncMoveDistance(motorGroup, motor_count, distance);
  * status = 1; //1 means it is finished

}
*/
