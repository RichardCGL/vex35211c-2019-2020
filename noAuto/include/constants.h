#pragma once
#include "vex.h"
#include "robot-config.h"

const float WHEEL_DIAMETER = 4; //inches 
const float PI = 3.1415926;
const float WHEEL_CIECUMFRENCE = PI * WHEEL_DIAMETER;
const float WHEEL_CIECUMFRENCE_ADJ = WHEEL_CIECUMFRENCE + 0;//adjusted circumference based on testing
const float TRACK_WIDTH = 4.75;//width between two side 
const float WHEEL_BASE= 5.625;

competition Competition;
const int WHEELF_COUNT = 2;
motor_group wheelF[WHEELF_COUNT] = {wheelLF,wheelRF};
const int WHEELB_COUNT = 2;
motor_group wheelB[WHEELB_COUNT] = {wheelLB,wheelRB};
const int WHEELL_COUNT= 2;
motor_group wheelL[WHEELL_COUNT] = {wheelLF,wheelLB};
const int WHEELR_COUNT = 2;
motor_group wheelR[WHEELR_COUNT] = {wheelRF,wheelRB};
const int INTAKE_COUNT = 2;
motor_group intake[INTAKE_COUNT] = {intakeMotorL,intakeMotorR};
const int MOTOR_COUNT = 8;
motor_group allMotors[MOTOR_COUNT] = {wheelLF,   wheelLB,    wheelRF,    wheelRB,
                                //1       2           3           4
                                rise,   push,       intakeMotorL,intakeMotorR};
                                //5       6           7           8


motor_group base_L(wheelLB, wheelLF);
motor_group base_R(wheelRB, wheelRF);
drivetrain base_train(base_L, base_R, WHEEL_CIECUMFRENCE_ADJ, TRACK_WIDTH, WHEEL_BASE, distanceUnits::in);

