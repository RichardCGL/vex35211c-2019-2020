#pragma once

void pidMove(int, vex::motor);
void motorGroupSpin(vex::motor_group [], vex::directionType, double,vex::velocityUnits);
void motorGroupStop(vex::motor_group [], vex::brakeType);
void goToTime(int, int);
void stopBot(void);
void statsMonitor(void);
void asyncMoveDistance(vex::motor_group [], float, int *);

