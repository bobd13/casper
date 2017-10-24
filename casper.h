/* casper.h - bot based on sprout



*/

#ifndef _CASPER_H_
#define _CASPER_H_

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include <math.h>
#include <signal.h>
#include <string.h>
#include <SDL2/SDL.h>   /* All SDL App's need this */
#include "../Adafruit_Motor_Shield_V2_library/Adafruit_MotorShield.h"
#include "../Adafruit_Motor_Shield_V2_library/utility/Adafruit_MS_PWMServoDriver.h"

// should be 192 if it is 48:1 gear ratio
// right:
#define TICKS_PER_REV 195
// left:
// #define TICKS_PER_REV 195
#define CIRCUMFERENCE_IN 8.125
#define TICKS_PER_INCH (CIRCUMFERENCE_IN / (1.0 * TICKS_PER_REV))

#define JOYSTICK_MAX 32767

void delay(long ms);
void run_to_distance_inches(FILE *fenc, double ldist_in, double rdist_in, int lspeed_max, int rspeed_max);
void run_to_distance_ticks(FILE *fenc, long ldist, long rdist, int lspeed_max, int rspeed_max);
void exiting(void);
void signal_handler(int signal_number);

#endif
