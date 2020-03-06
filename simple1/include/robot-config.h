#pragma once

using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern motor wheelLF;
extern motor wheelLB;
extern motor wheelRF;
extern motor wheelRB;
extern motor rise;
extern motor push;
extern motor intakeMotorL;
extern motor intakeMotorR;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Text.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );