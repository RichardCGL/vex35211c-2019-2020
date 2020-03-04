/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       vex.h                                                     */
/*    Author:       Vex Robotics                                              */
/*    Created:      1 Feb 2019                                                */
/*    Description:  Default header for V5 projects                            */
/*                                                                            */
/*----------------------------------------------------------------------------*/
//
#pragma once
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "v5.h"
#include "v5_vcs.h"
#include "constants.h"







/*---------------------------------------------------------------------------*/
/*                              Constant Vables                              */
/*---------------------------------------------------------------------------*/

static const double threshold = 2;
static const double thresholdDeg = 10;
static const int timeDelay = 50; //50 ms delay after each loop (20Hz) //vex::task::sleep(timeDelay);
static const double MOTOR_MAX_TEMP = 60;
static const double CTR_TO_RPM = 1.5748;
static const double CTR_TO_RPM_LESS = 1.4;


/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Text.
 * 
 * This should be called at the start of your int main function.
 */

#define waitUntil(condition)                                                   \
  do {                                                                         \
    wait(5, msec);                                                             \
  } while (!(condition))

#define repeat(iterations)                                                     \
  for (int iterator = 0; iterator < iterations; iterator++)

#include "robot-config.h"

#define waitUntil(condition)                                                   \
  do {                                                                         \
    wait(5, msec);                                                             \
  } while (!(condition))

#define repeat(iterations)                                                     \
  for (int iterator = 0; iterator < iterations; iterator++)