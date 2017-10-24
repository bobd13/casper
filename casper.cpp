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


BUILD: g++ -o casper casper.cpp ../Adafruit_Motor_Shield_V2_library/Adafruit_MotorShield.o ../Adafruit_Motor_Shield_V2_library/utility/Adafruit_MS_PWMServoDriver.o -lm -SDL2


*/

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
#include "casper.h"

#define FILE_COUNTR "/dev/quad_enc"

#define P 1
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

void run_to_distance_inches(FILE *fenc, double ldist_in, double rdist_in, int lspeed_max, int rspeed_max)
{
	long ldist;
	long rdist;

	ldist = ldist_in * TICKS_PER_INCH;
	rdist = rdist_in * TICKS_PER_INCH;

	run_to_distance_ticks(fenc, ldist, rdist, lspeed_max, rspeed_max);
}

void run_to_distance_ticks(FILE *fenc, long ldist, long rdist, int lspeed_max, int rspeed_max)
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

	motorLeft->setSpeed(lspeed_max);
  motorRight->setSpeed(rspeed_max);

	// do all the calculations with the double versions of speed, then change to int before setting motors
	countl = 0;
  countr = 0;
  while((countr < rdist) || (countl < ldist)) {
    fscanf(fenc, "%ld %ld", &countr_raw, &countl_raw);
		countl = countl_raw - countl_0;
    countr = countr_raw - countr_0;
		printf("r: %ld  l: %ld\n", countr, countl);
		
		// 1st, PID the motors
    lspeed_d = P * (ldist - countl);
    rspeed_d = P * (rdist - countr);
		printf("1 rspeed: %lf  lspeed: %lf\n", rspeed_d, lspeed_d);
		
		// 2nd, adjust for speed difference of left and right motors
		speed_ratio = ((1.0 * countr)/(1.0 * rdist)) / ((1.0 * countl)/(1.0 * (ldist)));
		printf("speed_ratio: %lf\n", speed_ratio);
		lspeed_d *= speed_ratio;
		printf("2 rspeed: %lf  lspeed: %lf\n", rspeed_d, lspeed_d);
		
		// 3rd, limit the speed, but keep the balance by scaling the other motor also
    if(lspeed_d > lspeed_max) {
			// scale right speed to maintain balance above
			rspeed_d *= (1.0 * lspeed_max) / lspeed_d;
      lspeed_d = lspeed_max;
    }

    if(rspeed_d > rspeed_max) {
			// scale left speed to maintain balance above
			lspeed_d *= (1.0 * rspeed_max) / rspeed_d;
      rspeed_d = rspeed_max;
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

		lspeed = int(lspeed_d);
		rspeed = int(rspeed_d);
		motorLeft->setSpeed(lspeed);
    motorRight->setSpeed(rspeed);
    delay(10);
  }
  
  // turn off motor
	motorLeft->setSpeed(0);
	motorRight->setSpeed(0);

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
				if(event.jbutton.button == 3) {
					done = 1;
				}
				if(event.jbutton.button == 2) {
					run_to_distance_ticks(fid_countr, 5 * TICKS_PER_REV, 5 * TICKS_PER_REV, 50, 50);
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

	exit(0);


	/*
	countr = 0;
  target = REVOLUTIONS * TICKS_PER_REV;
  printf("target: %ld\n", target);
  while(countr < target) {
    fscanf(fid_countr, "%ld", &countr_raw);
    countr = countr_raw - countr_0;
    speed = P * (target - countr);
    if(speed > SPEED_MAX) {
      speed = SPEED_MAX;
    }
    if(speed < SPEED_MIN) {
      speed = SPEED_MIN;
    }
    // motorLeft->setSpeed(speed);
    motorRight->setSpeed(speed);
    delay(10);
    if(fabs(countr % 96) < 1) {
      printf("%ld\n", countr);
    }
  }
  
  // turn off motor
  motorLeft->run(RELEASE);
  motorRight->run(RELEASE);

  fscanf(fid_countr, "%ld", &countr_raw);
  countr = countr_raw - countr_0;
  printf("release: %ld\n", countr);

  delay(1000);

  fscanf(fid_countr, "%ld", &countr_raw);
  countr = countr_raw - countr_0;
  printf("%ld\n", countr);

  fcloseall();
  exit(0);
  

  // motorLeft->setSpeed(50);
  motorRight->setSpeed(50);
  motorLeft->run(FORWARD);
  motorRight->run(FORWARD);

  sleep(5);

  motorLeft->run(RELEASE);
  motorRight->run(RELEASE);

  fcloseall();
  exit(0);
  
  while(1) {
    uint8_t i;
  
    // Serial.print("tick");
    printf("tick\n");
    
    motorLeft->run(FORWARD);
    motorRight->run(FORWARD);
    for (i=0; i<100; i++) {
      motorLeft->setSpeed(i);  
      motorRight->setSpeed(i);  
      delay(10);
    }
    delay(100);
    for (i=100; i!=0; i--) {
      motorLeft->setSpeed(i);  
      motorRight->setSpeed(i);  
      delay(10);
    }
  
    // Serial.print("tock");
    printf("tock\n");
    
    motorLeft->run(BACKWARD);
    motorRight->run(BACKWARD);
    for (i=0; i<100; i++) {
      motorLeft->setSpeed(i);  
      motorRight->setSpeed(i);  
      delay(10);
    }
    delay(100);
    for (i=100; i!=0; i--) {
      motorLeft->setSpeed(i);  
      motorRight->setSpeed(i);  
      delay(10);
    }

    // Serial.print("tech");
    printf("tech\n");
    motorLeft->run(RELEASE);
    motorRight->run(RELEASE);
    delay(1000);
  }
	*/
}

