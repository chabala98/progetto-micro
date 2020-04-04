#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h>
#include <main.h>
#include <motors.h>
//#include <camera/po8030.h>
#include <chprintf.h>



#include "sensors/imu.h"
#include "sensors/VL53L0X/VL53L0X.h"
#include "cmd.h"
#include "config_flash_storage.h"
#include "exti.h"
#include "i2c_bus.h"
#include <main.h>
#include "motors.h"
#include "communication.h"




static Robot robot;

void SendUint8ToComputer(uint8_t* data, uint16_t size) 
{
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)"START", 5);
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)&size, sizeof(uint16_t));
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)data, size);
}

static void serial_start(void)
{
	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};

	sdStart(&SD3, &ser_cfg); // UART3.
}

uint16_t retrieve_TOF_measure(uint16_t dist,uint8_t nbr_of_measures){
	dist = 0;
	for(int i =0; i<nbr_of_measures;i++){
		chThdSleepMilliseconds(150);
		dist+=VL53L0X_get_dist_mm();
	}
	dist = dist/nbr_of_measures;
	return dist;
}

uint16_t adjust_orientation(void){
	bool adjusted = false;
	uint16_t current_dist=0,next_dist=0,prev_dist=0;
	rotate_angle(-6,robot.orientation);
	prev_dist = retrieve_TOF_measure(prev_dist,3);
	rotate_angle(6,robot.orientation);
	current_dist = retrieve_TOF_measure(current_dist,3);
	while(!adjusted){
		rotate_angle(6,robot.orientation);
		next_dist = retrieve_TOF_measure(next_dist,3);
		chprintf((BaseSequentialStream *)&SD3, "prev: %d\r\n\n", prev_dist);
		chprintf((BaseSequentialStream *)&SD3, "curr: %d\r\n\n", current_dist);
		chprintf((BaseSequentialStream *)&SD3, "next: %d\r\n\n", next_dist);
		if((current_dist < prev_dist) && (current_dist < next_dist)){
			rotate_angle(-6,robot.orientation);
			adjusted = true;
		}
		//commands to analyze

		//rearange dist
		else{
			prev_dist = current_dist;
			current_dist = next_dist;
		}

	}
	return current_dist;
}


void move_robot(uint16_t desired_x, uint16_t desired_y,float desired_orientation){
	chprintf((BaseSequentialStream *)&SD3, "desired x: %d  desired y: %d	\r\n\n", desired_x, desired_y);
	int16_t delta_x = desired_x-robot.position.x;
	int16_t delta_y = desired_y-robot.position.y;
	float distance;
	float angle = 0;
	if(delta_x == 0 && delta_y == 0); // @suppress("Suspicious semicolon")
	else{
		angle = (float) ((float)delta_y/(float)delta_x);

		angle= atanf(angle)*180/M_PI;
		chprintf((BaseSequentialStream *)&SD3, "angle: %f	\r\n\n", angle);
		chprintf((BaseSequentialStream *)&SD3, "angle-robotorientation: %f	\r\n\n", (angle-robot.orientation));
		if((delta_x > 0 && delta_y > 0)||(delta_x > 0 && delta_y < 0))
			robot.orientation = rotate_angle((angle-robot.orientation),robot.orientation);
		else if((delta_x < 0 && delta_y >0)||(delta_x < 0 && delta_y < 0))
			robot.orientation = rotate_angle((180+angle-robot.orientation),robot.orientation);
		else if(delta_x == 0){
			if(desired_y > 0)
				robot.orientation = rotate_angle((90-robot.orientation),robot.orientation);
			else
				robot.orientation = rotate_angle((-90-robot.orientation),robot.orientation);
			}
		else if(delta_y == 0){
			if(desired_x > 0)
				robot.orientation = rotate_angle((-robot.orientation),robot.orientation);
			else
				robot.orientation = rotate_angle((180-robot.orientation),robot.orientation);
		}
		distance = (float)(sqrt(pow((double)delta_x,2)+pow((double)delta_y,2)));
		move(distance);
	}

	chprintf((BaseSequentialStream *)&SD3, "IN MOVE ROBOT delta x = %d, delta y = %d, angle: %f	\r\n\n",delta_x,delta_y, angle);
	chprintf((BaseSequentialStream *)&SD3, "INT MOVE ROBOT orientation = %f   x = %d    y = %d"  ,robot.orientation,robot.max.x,robot.max.y);
	robot.position.x = desired_x;
	robot.position.y = desired_y;
	robot.orientation = rotate_angle(0,robot.orientation); //initilizazione
	robot.orientation = rotate_angle(desired_orientation-robot.orientation,robot.orientation);
}





int main(void)
{

    halInit();
    chSysInit();
    mpu_init();

    //starts the serial communication
    serial_start();
    //start the USB communication
    usb_start();
    //starts the camera
    //dcmi_start();
	//po8030_start();
	//inits the motors
	motors_init();
	VL53L0X_start();



	//stars the threads for the pi regulator and the processing of the image
	//pi_regulator_start();
	//process_image_start();

    /* Infinite loop. */
    //while (1) {
	/*
    		chThdSleepMilliseconds(10000);
    		robot.max.x = adjust_orientation() + DIAMETER_ROBOT/2;
    		robot.position.x = robot.max.x;
    		robot.orientation = 0;
    		robot.orientation = rotate_angle(90,robot.orientation);
    		robot.max.y = retrieve_TOF_measure(robot.max.y,3)+ DIAMETER_ROBOT/2;
    		robot.position.y = robot.max.y;
    		robot.orientation = rotate_angle(90,robot.orientation);
    		robot.max.x += retrieve_TOF_measure(robot.max.x,3)+ DIAMETER_ROBOT/2;
    		robot.orientation = rotate_angle(90,robot.orientation);
    		robot.max.y += retrieve_TOF_measure(robot.max.y,3)+ DIAMETER_ROBOT/2;
    		robot.orientation = rotate_angle(90,robot.orientation);
    		chprintf((BaseSequentialStream *)&SD3, "IN MAIN: orientation = %f   x = %d   y = %d",robot.orientation,robot.max.x,robot.max.y);
    		chprintf((BaseSequentialStream *)&SD3, "IN MAIN: coord x = %d   coord y = %d ",robot.position.x,robot.position.y);
    		move_robot(robot.position.x,robot.position.y,robot.orientation);
    		move_robot((uint16_t)(robot.max.x/2), (uint16_t)(robot.max.y/2),0);

    		//uint16_t current_dist,next_dist,prev_dist;
    		 */

    		robot.position.x = 0;
    		robot.orientation = 0;
    		robot.max.x = 0;
    		robot.max.y = 0;
    		robot.position.y = 0;
    		move_robot(0,0,0);
    		chThdSleepMilliseconds(10000);
    		move_robot(400,400,0);
    		chThdSleepMilliseconds(10000);
       	move_robot(-400,400,0);
        	chThdSleepMilliseconds(10000);
       	move_robot(-400,-400,0);
        	chThdSleepMilliseconds(10000);
   		move_robot(400,-400,0);
    		chThdSleepMilliseconds(10000);

    		//chprintf((BaseSequentialStream *)&SD3, "angle-robotorientation: %f	\r\n\n", (robot.orientation));
    		//chThdSleepMilliseconds(1000);

    		//move_robot(400, 400,90);
    		/*
    		robot.orientation = rotate_angle(0,robot.orientation);
    		chprintf((BaseSequentialStream *)&SD3, "angle-robotorientation: %f	\r\n\n", (robot.orientation));
    		chThdSleepMilliseconds(1000);
    		robot.orientation = rotate_angle(90,robot.orientation);
    		chprintf((BaseSequentialStream *)&SD3, "angle-robotorientation: %f	\r\n\n", (robot.orientation));
    		chThdSleepMilliseconds(1000);
    		robot.orientation = rotate_angle(180,robot.orientation);
    		chprintf((BaseSequentialStream *)&SD3, "angle-robotorientation: %f	\r\n\n", (robot.orientation));
    		chThdSleepMilliseconds(1000);
    		robot.orientation = rotate_angle(360,robot.orientation);
    		chprintf((BaseSequentialStream *)&SD3, "angle-robotorientation: %f	\r\n\n", (robot.orientation));
    		chThdSleepMilliseconds(1000);
    		robot.orientation = rotate_angle(45,robot.orientation);
    		chprintf((BaseSequentialStream *)&SD3, "angle-robotorientation: %f	\r\n\n", (robot.orientation));
    		chThdSleepMilliseconds(1000);
    		robot.orientation = rotate_angle(45,robot.orientation);
    		chprintf((BaseSequentialStream *)&SD3, "angle-robotorientation: %f	\r\n\n", (robot.orientation));
    		chThdSleepMilliseconds(1000);
    		robot.orientation = rotate_angle(270,robot.orientation);
    		chprintf((BaseSequentialStream *)&SD3, "angle-robotorientation: %f	\r\n\n", (robot.orientation));
    		chThdSleepMilliseconds(1000);
    		robot.orientation = rotate_angle(270,robot.orientation);
    		chprintf((BaseSequentialStream *)&SD3, "angle-robotorientation: %f	\r\n\n", (robot.orientation));
*/




    		/*
    		rotate_angle(-6);
    		prev_dist = VL53L0X_get_dist_mm();
    		chThdSleepMilliseconds(1000);
    		rotate_angle(6);
    		current_dist = VL53L0X_get_dist_mm();
    		chThdSleepMilliseconds(1000);
    		rotate_angle(6);
    		next_dist = VL53L0X_get_dist_mm();
    		chprintf((BaseSequentialStream *)&SD3, "prev: %d\r\n\n", prev_dist);
    		chprintf((BaseSequentialStream *)&SD3, "curr: %d\r\n\n", current_dist);
    		chprintf((BaseSequentialStream *)&SD3, "next: %d\r\n\n", next_dist);
    		*/
    	//waits 1 second
   // }

}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
