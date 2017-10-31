/* casper.cpp - bot based on sprout

some content from:

This is a test sketch for the Adafruit assembled Motor Shield for Arduino v2
It won't work with v1.x motor shields! Only for the v2's with built in PWM
control

For use with the Adafruit Motor Shield v2 
---->	http://www.adafruit.com/products/1438

10/15/17 BD
- from sprout.cpp and joystick.c

10/19/17 BD
- add run_to_distance_ticks() and run_to_distance_in()
- looks good at slow speed, but quite a bit off at high speed
- speed_ratio should be a PID output based on delta, need more gain when further apart

10/24/17 BD
- begin adding PID against target location
Start with assumed straight line
- add gettime_d()
- within 1 tick of each other at speed (ticks/sec) of 100, 200
- 500 ticks/sec - within 1 count until last few loops, then up to 2, but coasted to 3
- looks like right side reached target and started to slow, left coasts more
- need brake

10/26/17 BD
- running on the floor, the right wheel seemed too powerful and slipped on atartup
The tail hook caught on tiles and made strong steer
- glued half a ping-pog ball to tail hook
Now runs pretty straight and consistent in the hall
only tried with the bot centered left-right on a tile
- remove run_to_distance_inches_old() and run_to_distance_ticks_old
- attempt circle by scaling ldist and rdist for internal and external radii
- wheel separation (center to center) 5.75"
- need to calc left and right target distances using separate speeds in run_to_distance_ticks()
- while testing, noticed doesn't always complete (hear whining, no response to button push)
- basically works, runs a little too far

TODO:
- measure circle diameter and compare to plan
- add turn_left() or something similar
- add timer or something to get out of run_to... that doesn't complete
- rewrite such that run_to_... is called once per main loop
- put together whole drive path!

next up: will add run_arc_ticks() and run_arc_inches()

10/29/17
- added BRAKE to Adafruit Motorshield run()
- add BRAKE to run_to_distance_ticks()
- stops straighter
- twists when startsup sometimes, also when catches on a tile
- speed was 500, drop to 250
- 90 degree arc looks good - a little too long, but stops abruptly

10/20/17
- add swivel rear wheel based on pinewood derby car wheel
- still an issue
- raise speed to 500 from 250 and check


BUILD: g++ -o casper casper.cpp ../Adafruit_Motor_Shield_V2_library/Adafruit_MotorShield.o ../Adafruit_Motor_Shield_V2_library/utility/Adafruit_MS_PWMServoDriver.o -lm -SDL2


*/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include <math.h>
#include <signal.h>
#include <string.h>
#include <sys/time.h>
#include <SDL2/SDL.h>   /* All SDL App's need this */
#include "../Adafruit_Motor_Shield_V2_library/Adafruit_MotorShield.h"
#include "../Adafruit_Motor_Shield_V2_library/utility/Adafruit_MS_PWMServoDriver.h"
#include "casper.h"

#define FILE_COUNTR "/dev/quad_enc"

#define P 1
#define I 0.001

#define SPEED_MAX 255
#define SPEED_MIN 20

#define JOYSTICK_MAX 32767

Adafruit_MotorShield AFMS; 
Adafruit_DCMotor *motorRight;
Adafruit_DCMotor *motorLeft;
SDL_Joystick *joystick = 0;


void delay(long ms)
{
  struct timespec ts;

  ts.tv_sec = ms / 1000;
  ts.tv_nsec = (ms - ts.tv_sec * 1000) * 1000000;
  nanosleep(&ts, NULL);
}

double gettime_d()
{
	double t0;
	struct timeval tv0;

	gettimeofday(&tv0, NULL);
	t0 = 1.0 * tv0.tv_sec + (1.0 * tv0.tv_usec) / 1000000.0;
	// printf("seconds: %ld\n", tv0.tv_sec);
	// printf("usecs:   %ld\n", tv0.tv_usec);
	// printf("time:    %lf\n", t0);

	return t0;
}

void run_to_distance_inches(FILE *fenc, double ldist_in, double rdist_in, double speed_in)
{
	long ldist;
	long rdist;
	double speed;
	
	printf("run_to_distance_inches(l: %8.6lf r:%8.6lf s:%8.6lf)\n", ldist_in, rdist_in, speed_in);
	speed = speed_in * TICKS_PER_INCH;
	ldist = ldist_in * TICKS_PER_INCH;
	rdist = rdist_in * TICKS_PER_INCH;

	run_to_distance_ticks(fenc, ldist, rdist, speed);
}

/*
	speed is in ticks / second
 */
void run_to_distance_ticks(FILE *fenc, long ldist, long rdist, double speed)
{
  long countr_0;
  long countl_0;
  long countr;
  long countl;
  long countr_raw;
  long countl_raw;
  int i;
  int lspeed = 0;
  int rspeed = 0;
	double lspeed_d = 0.0;
	double rspeed_d = 0.0;
	double speed_ratio; // multiplier for lspeed to balance motors
	double t0;
	double tnow;
	long rdist_now;
	long ldist_now;
	double lIsum;
	double rIsum;
	double lerr;
	double rerr;
	double lspeed_target;
	double rspeed_target;
	double avg_speed_target;
	
	printf("run_to_distance_ticks(l: %6ld r:%6ld s:%8.6lf)\n", ldist, rdist, speed);
	if(ldist == rdist) {
		lspeed_target = speed;
		rspeed_target = speed;
	} else if(ldist == 0) {
		lspeed_target = 0;
		rspeed_target = speed;
	} else if(rdist == 0) {
		lspeed_target = speed;
		rspeed_target = 0;
	} else if(ldist > rdist) {
		lspeed_target = speed;
		rspeed_target = speed * ((1.0 * rdist) / (1.0 * ldist));
	} if(ldist < rdist) {
		lspeed_target = speed * ((1.0 * ldist) / (1.0 * rdist));
		rspeed_target = speed;
	}

	fscanf(fenc, "%ld %ld", &countr_0, &countl_0);
  printf("countr_0: %ld countl_0: %ld\n", countr_0, countl_0);
  
	if(ldist >= 0) {
		motorLeft->run(FORWARD);
	} else {
		motorLeft->run(BACKWARD);
	}
	if(rdist >= 0) {
		motorRight->run(FORWARD);
	} else {
		motorRight->run(BACKWARD);
	}

	motorLeft->setSpeed(lspeed);
  motorRight->setSpeed(rspeed);

	// do all the calculations with the double versions of speed, then change to int before setting motors
	countl = 0;
  countr = 0;
	t0 = gettime_d();
	tnow = t0;
	lerr = rerr = 0.0;
	lIsum = rIsum = 0.0;
  while((countr < rdist) || (countl < ldist)) {
    fscanf(fenc, "%ld %ld", &countr_raw, &countl_raw);
		countl = countl_raw - countl_0;
    countr = countr_raw - countr_0;
		printf("r: %ld  l: %ld\n", countr, countl);

		// where should it be now?
		tnow = gettime_d();
		ldist_now = lspeed_target * (tnow - t0);
		if(ldist_now > ldist) {
			ldist_now = ldist;
		}
		rdist_now = rspeed_target * (tnow - t0);
		if(rdist_now > rdist) {
			rdist_now = rdist;
		}
		
		// 1st, PID the motors
		lerr = ldist_now - countl;
		rerr = rdist_now - countr;
		lIsum += lerr;
		rIsum += rerr;
    lspeed_d = P * lerr + I * lIsum;
    rspeed_d = P * rerr + I * rIsum;
		printf("1 rspeed: %lf  lspeed: %lf\n", rspeed_d, lspeed_d);

		/*
		// 2nd, adjust for speed difference of left and right motors
		speed_ratio = ((1.0 * countr)/(1.0 * rdist)) / ((1.0 * countl)/(1.0 * (ldist)));
		printf("speed_ratio: %lf\n", speed_ratio);
		lspeed_d *= speed_ratio;
		printf("2 rspeed: %lf  lspeed: %lf\n", rspeed_d, lspeed_d);
		
		// 3rd, limit the speed, but keep the balance by scaling the other motor also
    if(lspeed_d > SPEED_MAX) {
			// scale right speed to maintain balance above
			rspeed_d *= (1.0 * SPEED_MAX) / lspeed_d;
      lspeed_d = SPEED_MAX;
    }

    if(rspeed_d > SPEED_MAX) {
			// scale left speed to maintain balance above
			lspeed_d *= (1.0 * SPEED_MAX) / rspeed_d;
      rspeed_d = SPEED_MAX;
    }
		printf("3 rspeed: %lf  lspeed: %lf\n", rspeed_d, lspeed_d);

		// finally, make sure motors still running >= min
    if(lspeed_d < SPEED_MIN) {
			// scale right speed to maintain balance above
			// rspeed_d *= (1.0 * SPEED_MIN) / lspeed_d;
      lspeed_d = SPEED_MIN * speed_ratio;
    }

    if(rspeed_d < SPEED_MIN) {
			// scale left speed to maintain balance above
			// lspeed_d *= (1.0 * SPEED_MIN) / rspeed_d;
      rspeed_d = SPEED_MIN;
    }
		printf("4 rspeed: %lf  lspeed: %lf\n", rspeed_d, lspeed_d);
		*/
		
		lspeed = int(lspeed_d);
		rspeed = int(rspeed_d);
		// Limit them
		if(lspeed > SPEED_MAX) {
			lspeed = SPEED_MAX;
		}
		if(rspeed > SPEED_MAX) {
			rspeed = SPEED_MAX;
		}
		motorLeft->setSpeed(lspeed);
    motorRight->setSpeed(rspeed);

		if(countl >= ldist) {
			motorLeft->setSpeed(0);
			motorLeft->run(BRAKE);
		}

		if(countr >= rdist) {
			motorRight->setSpeed(0);
			motorRight->run(BRAKE);
		}

		delay(10);
  }
  
  // turn off motor
	motorLeft->setSpeed(0);
	motorLeft->run(BRAKE);
	motorRight->setSpeed(0);
	motorRight->run(BRAKE);

  fscanf(fenc, "%ld %ld", &countr_raw, &countl_raw);
  countr = countr_raw - countr_0;
  countl = countl_raw - countl_0;
  printf("moved: %ld %ld\n", countr, countl);

  delay(1000);

  fscanf(fenc, "%ld %ld", &countr_raw, &countl_raw);
  countr = countr_raw - countr_0;
  countl = countl_raw - countl_0;
  printf("final moved: %ld %ld\n", countr, countl);

}

void exiting(void)
{
  printf("exiting...\n");
  fcloseall();
  
  // turn off motor
  motorLeft->run(RELEASE);
  motorRight->run(RELEASE);

	if(joystick) {
		SDL_JoystickClose(joystick);
	}
	// Shutdown all SDL subsystems 
	SDL_Quit();
}

void signal_handler(int signal_number)
{
  printf("Received signal(%d): %s\n", signal_number, strsignal(signal_number));
  exit(-1);
}

main(int argc, char *argv[]) {
  FILE *fid_countr;
  long countr_0;
  long countr_1;
  long countr;
  long countr_raw;
  int i;
  int lspeed = 0;
	int rspeed = 0;
	int x = 0;
	int y = 0;
  long target;
	int njoysticks;
	// int v = 0;
	// int w = 0;
	double k = 1;
	double labs;
	double rabs;
	double absmax;
	
	




	printf("Initializing SDL.\n");
    
	if (SDL_Init( SDL_INIT_JOYSTICK ) < 0) {
		fprintf(stderr, "Couldn't initialize SDL: %s\n", SDL_GetError());
		exit(1);
	}

	printf("SDL initialized.\n");

  // Create the motor shield object with the default I2C address
  AFMS = Adafruit_MotorShield(); 
  // Or, create it with a different I2C address (say for stacking)
  // Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61); 

  // Select which 'port' M1, M2, M3 or M4. In this case, M1
  motorRight = AFMS.getMotor(1);
  motorLeft = AFMS.getMotor(4);
  // You can also make another motor on port M2
  //Adafruit_DCMotor *myOtherMotor = AFMS.getMotor(2);

  atexit(exiting);
  signal(SIGINT, signal_handler); // ^C
  signal(SIGTERM, signal_handler); //  added these when above didn't appear to work
  signal(SIGKILL, signal_handler); //  added these when above didn't appear to work
  
  motorRight->swapConnections();

  if((fid_countr = fopen(FILE_COUNTR, "ro")) == NULL) {
    printf("Error - couldn't open %s\n", FILE_COUNTR);
    exit(-1);
  }

  fscanf(fid_countr, "%ld %ld", &countr_0, &countr_1);
  printf("countr_0: %ld  countr_1: %ld\n", countr_0, countr_1);
  
  AFMS.begin();  // create with the default frequency 1.6KHz
  //AFMS.begin(1000);  // OR with a different frequency, say 1KHz
  
  // Set the speed to start, from 0 (off) to 255 (max speed)
	motorLeft->setSpeed(0);
  motorRight->setSpeed(0);
  motorLeft->run(FORWARD);
  motorRight->run(FORWARD);


	njoysticks = SDL_NumJoysticks();
	if(njoysticks < 1) {
		printf("no joysticks found\n");
		exit(-1);
	}
	printf("%i joysticks were found.\n\n", njoysticks);
	printf("The names of the joysticks are:\n");
	for(i = 0; i < njoysticks; i++) {
		printf("    %s\n", SDL_JoystickNameForIndex(i));
	}

	SDL_JoystickEventState(SDL_ENABLE);
	// Go get the first joystick
	joystick = SDL_JoystickOpen(0);

	int number_of_buttons;
	number_of_buttons = SDL_JoystickNumButtons(joystick);
	printf("%d buttons\n", number_of_buttons);

	SDL_Event event;
	/* Other initializtion code goes here */   

	/* Start main game loop here */

	int done = 0;
	x = y = 0;
	while(!done) {
		if(SDL_PollEvent(&event)) {
			switch(event.type) {  
			case SDL_JOYAXISMOTION:  /* Handle Joystick Motion */
				if(event.jaxis.axis == 0) {
					/* Left-right movement code goes here */
					x = event.jaxis.value;
					printf("x: %d\n", x);
				}

				if( event.jaxis.axis == 1) {
					/* Up-Down movement code goes here */
					y = -event.jaxis.value;
					printf("\ty: %d\n", y);
				}
				break;

			case SDL_JOYBUTTONDOWN:  /* Handle Joystick Button Presses */
				/* code goes here */
				printf("button %d pressed\n", event.jbutton.button);

				if(event.jbutton.button == 0) {
					// left turn
					double radius = 24;
					double lradius = radius - LR_WIDTH/2.0; 
					double rradius = radius + LR_WIDTH/2.0;
					// 90 degrees
					double ldist = lradius * M_PI_2; 
					double rdist = rradius * M_PI_2; 
					run_to_distance_inches(fid_countr, ldist, rdist, 20);
				}

				if(event.jbutton.button == 2) {
					run_to_distance_ticks(fid_countr, 5 * TICKS_PER_REV, 5 * TICKS_PER_REV, 500);
					// run_to_distance_ticks(fid_countr, 5 * TICKS_PER_REV, 5 * TICKS_PER_REV, 250);
				}

				if(event.jbutton.button == 3) {
					done = 1;
				}
				break;

			case SDL_JOYBUTTONUP:  /* Handle Joystick Button Presses */
				/* code goes here */
				printf("button %d released\n", event.jbutton.button);
				break;

			case SDL_KEYDOWN:
				/* handle keyboard stuff here */				
				printf("key pressed\n");
				done = 1;
				break;

			case SDL_QUIT:
				/* Set whatever flags are necessary to */
				/* end the main game loop here */
				done = 1;
				break;
			}
		}

		// calculate speeds from joystick
		/* for test:
		lspeed = x * SPEED_MAX / JOYSTICK_MAX;
		rspeed = y * SPEED_MAX / JOYSTICK_MAX;
		*/
		/*
			from http://home.kendra.com/mauser/joystick.html
		V =(100-ABS(X)) * (Y/100) + Y;
		W= (100-ABS(Y)) * (X/100) + X;
		R = (V+W) /2;
		L= (V-W)/2;
		

		v = (100 - abs(x)) * (y/100) + y;
		w = (100 - abs(y)) * (x/100) + x;
		rspeed = (v + w) / 2;
		lspeed = (v - w) / 2;
		*/

		/*
			https://electronics.stackexchange.com/questions/19669/algorithm-for-mixing-2-axis-analog-input-to-control-a-differential-motor-drive
		*/
		// NO - scaling should be constant, I think
		lspeed = y + x;
		rspeed = y - x;
		
		labs = fabs(lspeed);
		rabs = fabs(rspeed);
		absmax = (labs > rabs) ? labs : rabs;
		k = 1.0;
		if(absmax > SPEED_MAX) {
			k = 1.0 * SPEED_MAX / absmax;
		}
		
		k = 0.004;
		
		lspeed *= k;
		rspeed *= k;

		fscanf(fid_countr, "%ld %ld", &countr_0, &countr_1);
		printf("k: %6lf  x: %4d   y: %4d    l: %4d   r: %4d   c0:%8ld   c1:%8ld\n", k, x, y, lspeed, rspeed, countr_0, countr_1);
		if(lspeed > 0) {
			motorLeft->run(FORWARD);
		} else {
			motorLeft->run(BACKWARD);
		}
		if(rspeed > 0) {
			motorRight->run(FORWARD);
		} else {
			motorRight->run(BACKWARD);
		}
    motorLeft->setSpeed(abs(lspeed));
    motorRight->setSpeed(abs(rspeed));
		
		delay(10);

	}

	/* End loop here */


}

