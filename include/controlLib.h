#pragma once
#include "vex.h"

void pidMove(int, motor);
void motorGroupSpin(motor_group [], directionType, double,velocityUnits);
void motorGroupStop(motor_group [], brakeType);
void goToTime(int, int);
void stopBot(void);
void statsMonitor(void);
//void asyncMoveDistance(motor_group [], float, int *, int);
