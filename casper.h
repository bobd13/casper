/* casper.h - bot based on sprout

10/24/17
- add gettime_d()
10/26/17 BD
- remove run_to_distance_inches_old() and run_to_distance_ticks_old
- add LR_WIDTH

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
#define TICKS_PER_INCH ((1.0 * TICKS_PER_REV) / CIRCUMFERENCE_IN)
#define LR_WIDTH 5.75

#define JOYSTICK_MAX 32767

void delay(long ms);
double gettime_d();
void run_to_distance_inches(FILE *fenc, double ldist_in, double rdist_in, double speed_in);
void run_to_distance_ticks(FILE *fenc, long ldist, long rdist, double speed);
void exiting(void);
void signal_handler(int signal_number);

#endif
