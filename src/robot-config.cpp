#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor wheelLF = motor(PORT11, ratio18_1, false);
motor wheelLB = motor(PORT12, ratio18_1, false);
motor wheelRF = motor(PORT14, ratio18_1, true);
motor wheelRB = motor(PORT15, ratio18_1, true);
motor rise = motor(PORT13, ratio36_1, false);
motor push = motor(PORT16, ratio36_1, true);
motor intakeMotorL = motor(PORT17, ratio36_1, false);
motor intakeMotorR = motor(PORT18, ratio36_1, true);

// VEXcode generated functions



/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Text.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}