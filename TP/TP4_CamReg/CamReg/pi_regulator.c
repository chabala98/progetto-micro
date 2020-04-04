#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>


#include <main.h>
#include <motors.h>
#include <pi_regulator.h>
#include <process_image.h>
#define DESIRED_DISTANCE 10
#define NSTEP_ONE_TURN 1000
#define WHEEL_PERIMETER 13
#define MAX_SPEED 2200




static THD_WORKING_AREA(waPiRegulator, 256);
static THD_FUNCTION(PiRegulator, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time = 0;

    int16_t speed = 0;

	float distance;
	float error;
	float previous_error;
	float integral = 0;
	float Kp = 5, Ki = 0.1;
	float dt;

    while(1){
        dt = chVTGetSystemTime()-time;
        time = chVTGetSystemTime();
        distance = get_distance_cm();
        	error = distance - DESIRED_DISTANCE;

        /*
		*	To complete
		*/

        	integral = integral + error*dt;
        	if((previous_error < 0 && error > 0) || (previous_error > 0 && error<0))
        	{
        		integral = 0;
        		error = 0;
        	}
        	if(fabs(integral*Ki) > MAX_SPEED/2)
        		integral = MAX_SPEED/2/Ki/dt;

        speed = ((Kp*error + Ki*integral)/dt)* NSTEP_ONE_TURN / WHEEL_PERIMETER;//(1000/(2*3.14*13));
        previous_error = error;
        //applies the speed from the PI regulator
		right_motor_set_speed(speed);
		left_motor_set_speed(speed);

        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(100));
    }
}

void pi_regulator_start(void){
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, NULL);
}
