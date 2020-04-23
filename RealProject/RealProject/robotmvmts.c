#include "motors.h"
#include <chprintf.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h>
#include "communication.h"
#include <robotmvmts.h>
#include <main.h>
#include "sensors/VL53L0X/VL53L0X.h"
#include "process_image.h"
#include "leds.h"

/***************************INTERNAL FUNCTIONS************************************/
static Robot robot;
static Goal goals[NB_GOALS];




uint16_t retrieve_TOF_measure(uint8_t nbr_of_measures){
	uint16_t dist = 0;
	for(int i =0; i<nbr_of_measures;i++){
		chThdSleepMilliseconds(150);
		dist+=VL53L0X_get_dist_mm();
	}
	dist = dist/nbr_of_measures;
	return dist;
}


void adjust_orientation(){
	uint16_t dist1=0,dist2=0;
	float angle = 0;
	dist1 = retrieve_TOF_measure(5);
	rotate_angle(10,robot.orientation);
	dist2 = retrieve_TOF_measure(5);
	// we calculate the denominator so that we don't do it repeatidly during the function (less calculation time in total)
	float denominator = sqrtf((dist1*dist1)-(float)(2*(float)dist1*(float)dist2*COS10CTE)+dist2*dist2);
	angle = atan2f((dist1-(float)((dist2*COS10CTE))/denominator), (-dist2*SIN10CTE)/denominator) *180/M_PI;
	if(angle > 0)
		rotate_angle(-10-angle+180,robot.orientation);
	else
		rotate_angle(-10-angle-180,robot.orientation);
	//chprintf((BaseSequentialStream *)&SD3, "angle:  %f	\r\n\n", angle);
	/*
	bool adjusted = false;
	uint16_t current_dist=0,next_dist=0, next_next_dist = 0,prev_dist=0,prev_prev_dist=0;
	rotate_angle(-angle_precision,robot.orientation);
	prev_dist = retrieve_TOF_measure(5);
	rotate_angle(-angle_precision,robot.orientation);
	prev_prev_dist = retrieve_TOF_measure(5);
	rotate_angle(2*angle_precision,robot.orientation);
	current_dist = retrieve_TOF_measure(5);
	rotate_angle(angle_precision,robot.orientation);
	next_dist = retrieve_TOF_measure(5);
	rotate_angle(angle_precision,robot.orientation);
	next_next_dist = retrieve_TOF_measure(5);
	rotate_angle(-2*angle_precision,robot.orientation);
	while(!adjusted){
		if((current_dist <= prev_dist) && (current_dist <= prev_prev_dist)  && (current_dist <= next_dist) && (current_dist <= next_next_dist))
			adjusted = true;
		//commands to analyze
		//rearange dist
		else{
			if((next_dist < current_dist)  && (prev_dist > current_dist)){
				if(next_next_dist < next_dist){
					prev_prev_dist = prev_dist;
					prev_dist = current_dist;
					current_dist = next_dist;
					next_dist = next_next_dist;
					rotate_angle(3*angle_precision,robot.orientation);
					next_next_dist = retrieve_TOF_measure(5);
					rotate_angle(-2*angle_precision,robot.orientation);
				}
				else{
					rotate_angle(angle_precision,robot.orientation);
					adjusted = true;
				}
			}
			else{
				if(prev_prev_dist < prev_dist){
					next_next_dist = next_dist;
					next_dist = current_dist;
					current_dist = prev_dist;
					prev_dist = prev_prev_dist;
					rotate_angle(-3*angle_precision,robot.orientation);
					prev_prev_dist = retrieve_TOF_measure(5);
					rotate_angle(2*angle_precision,robot.orientation);
				}
				else{
					rotate_angle(-angle_precision,robot.orientation);
					adjusted = true;
				}
			}
		}

	}
	return current_dist;
	*/
}

void move_robot(uint16_t desired_x, uint16_t desired_y,float desired_orientation,bool remeasure_pos){
	//chprintf((BaseSequentialStream *)&SD3, "desired x: %d  desired y: %d	\r\n\n", desired_x, desired_y);
	if(remeasure_pos){
		move_robot(robot.position.x,robot.position.y,180,false);
		adjust_orientation();
		robot.orientation = 180;
		robot.position.x = retrieve_TOF_measure(10);
		move_robot(robot.position.x,robot.position.y,-90,false);
		adjust_orientation();
		robot.orientation = -90;
		robot.position.y = retrieve_TOF_measure(10);
	}

	int16_t delta_x = desired_x-robot.position.x;
	int16_t delta_y = desired_y-robot.position.y;

	float distance = 0;
	float angle = 0;
	//static int i = 0;
	if(delta_x == 0 && delta_y == 0); // @suppress("Suspicious semicolon")
	else{
		if(delta_x != 0){
			angle= atanf((float)((float)delta_y/(float)delta_x))*180/M_PI;
		}

		//chprintf((BaseSequentialStream *)&SD3, "angle: %f	\r\n\n", angle);
		//chprintf((BaseSequentialStream *)&SD3, "angle-robotorientation: %f	\r\n\n", (angle-robot.orientation));
		if((delta_x > 0 && delta_y > 0)||(delta_x > 0 && delta_y < 0))
			robot.orientation = rotate_angle((angle-robot.orientation),robot.orientation);
		else if((delta_x < 0 && delta_y >0)||(delta_x < 0 && delta_y < 0))
			robot.orientation = rotate_angle((180+angle-robot.orientation),robot.orientation);
		else if(delta_x == 0){
			if(delta_y > 0)
				robot.orientation = rotate_angle((90-robot.orientation),robot.orientation);
			else
				robot.orientation = rotate_angle((-90-robot.orientation),robot.orientation);
			}
		else if(delta_y == 0){
			if(delta_x > 0)
				robot.orientation = rotate_angle((-robot.orientation),robot.orientation);
			else
				robot.orientation = rotate_angle((180-robot.orientation),robot.orientation);
		}

		//chprintf((BaseSequentialStream *)&SD3, "distance : %f \r\n\n",distance);
		distance = (float)(sqrtf(((delta_x*delta_x)+(delta_y*delta_y))));
		//chprintf((BaseSequentialStream *)&SD3, "distance: %f	\r\n\n", distance);
		//chprintf((BaseSequentialStream *)&SD3, " %d	\r\n\n", i);
		move(distance);
	}
	chprintf((BaseSequentialStream *)&SD3, "angle: %f , deltax: %d,  deltay: %d	\r\n\n", angle,delta_x,delta_y);

	//chprintf((BaseSequentialStream *)&SD3, "IN MOVE ROBOT delta x = %d, delta y = %d, angle: %f	\r\n\n",delta_x,delta_y, angle);
	//chprintf((BaseSequentialStream *)&SD3, "INT MOVE ROBOT orientation = %f   x = %d    y = %d"  ,robot.orientation,robot.max.x,robot.max.y);
	robot.position.x = desired_x;
	robot.position.y = desired_y;
	robot.orientation = rotate_angle(0,robot.orientation); //initilizazione
	robot.orientation = rotate_angle(desired_orientation-robot.orientation,robot.orientation);
}
	/***************************END INTERNAL FUNCTIONS************************************/
uint8_t chose_color_to_analyse(uint8_t goal_color){
	switch(goal_color){
		case RED :
			return BLUE;
		default:
			return RED;
	}
}


void attribute_goal(void){
	//AGGIUSTARE LE ORIENTAZIONI
	uint8_t color_to_analyse;
	uint16_t line_position;
	int16_t dist_to_move;
	for(int i = FIRST_GOAL; i<3 ; i++)
		goals[i].color = COLOR_NOT_ATTRIBUTED;
	//SCAN FIRST WALL
	uint8_t i = FIRST_GOAL;
	bool identified_goal = false;
	move_robot(SECURE_DIST,DIST_CAMERA_MEASURE,-90,true);
	chThdSleepMilliseconds(1000);
	while(!identified_goal){
		chThdSleepMilliseconds(500);
		if(get_color_detected() != WHITE)
			identified_goal=true;
		else if((robot.position.x + STEP_POSITION) >= (robot.max.x - SECURE_DIST))
			break;
		else
			move_robot(robot.position.x + STEP_POSITION, DIST_CAMERA_MEASURE,-90,false);
		chThdSleepMilliseconds(1000);
	}
	goals[i].color = get_color_detected();
	//CHECKING IF COLOR WAS ATTRIBUTED TO GOAL
	if(goals[i].color == WHITE)
		goals[i].color = COLOR_NOT_ATTRIBUTED;
	else{
		light_rgb_led(goals[i].color);
		color_to_analyse = chose_color_to_analyse(goals[i].color);
		while(!(get_line_width(color_to_analyse) >0)){
			move_robot(robot.position.x, robot.position.y + STEP_POSITION,-90,true);
			chThdSleepMilliseconds(500);
		}
		line_position = get_line_position();
		//robot.position.y = retrieve_TOF_measure(10);
		dist_to_move = robot.position.y*TAN_CAMERA_APERTURE*(line_position-320)/320;
		move_robot(robot.position.x - dist_to_move,robot.position.y,-90,true);
		chThdSleepMilliseconds(5000);
		goals[i].position.x = robot.position.x;
		goals[i].position.y = 0;
		i++;
		light_rgb_led(OFF);
	}



    	//SCAN SECOND WALL
    	identified_goal = false;
    	move_robot(robot.max.x - DIST_CAMERA_MEASURE,SECURE_DIST,0,true);
    	chThdSleepMilliseconds(1000);
    	while(!identified_goal){
    		chThdSleepMilliseconds(500);
    		if(get_color_detected() != WHITE){
    			identified_goal=true;
    		}
    		else if((robot.position.y + STEP_POSITION) >= (robot.max.y - SECURE_DIST))
    			break;
    		else
    			move_robot(robot.max.x - DIST_CAMERA_MEASURE, robot.position.y + STEP_POSITION,0,false);
    		chThdSleepMilliseconds(1000);
    	}
    	goals[i].color = get_color_detected();

    	//CHECKING IF COLOR WAS ATTRIBUTED TO GOAL
    	if(goals[i].color == WHITE)
    		goals[i].color = COLOR_NOT_ATTRIBUTED;
    	else{
    		light_rgb_led(goals[i].color);
    		color_to_analyse = chose_color_to_analyse(goals[i].color);
    		while(!(get_line_width(color_to_analyse) > 0)){
    			move_robot(robot.position.x - STEP_POSITION, robot.position.y,0,true);
    			chThdSleepMilliseconds(500);
    		}
    		line_position = get_line_position();
    		//robot.position.x = robot.max.x-retrieve_TOF_measure(10);
    		dist_to_move = (robot.max.x-robot.position.x)*TAN_CAMERA_APERTURE*(line_position-320)/320;
    		move_robot(robot.position.x ,robot.position.y-dist_to_move,0,true);
    		chThdSleepMilliseconds(5000);
    		goals[i].position.x = robot.max.x;
    		goals[i].position.y = robot.position.y;
    		i++;
    		light_rgb_led(OFF);
    	}

    //SCAN THIRD GOAL
 	identified_goal = false;
 	move_robot((robot.max.x - SECURE_DIST),(robot.max.y - DIST_CAMERA_MEASURE),90,true);
 	chThdSleepMilliseconds(1000);
 	while(!identified_goal){
 		chThdSleepMilliseconds(500);
 		if(get_color_detected() != WHITE){
 			identified_goal=true;
 		}
 		else if((robot.position.x - STEP_POSITION) <= SECURE_DIST)
 			break;
 		else
 			move_robot(robot.position.x-STEP_POSITION,(robot.max.y - DIST_CAMERA_MEASURE),90,false);
 		chThdSleepMilliseconds(1000);
 	}
 	goals[i].color = get_color_detected();

 	//CHECKING IF COLOR WAS ATTRIBUTED TO GOAL
 	if(goals[i].color == WHITE)
 		goals[i].color = COLOR_NOT_ATTRIBUTED;
	else{
		light_rgb_led(goals[i].color);
		color_to_analyse = chose_color_to_analyse(goals[i].color);
		while(!(get_line_width(color_to_analyse) > 0)){
			move_robot(robot.position.x, robot.position.y-STEP_POSITION,90,true);
			chThdSleepMilliseconds(500);
		}
		line_position = get_line_position();
		//robot.position.y = robot.max.y - retrieve_TOF_measure(10);
		dist_to_move = (robot.max.y-robot.position.y)*TAN_CAMERA_APERTURE*(line_position-320)/320;
		move_robot(robot.position.x + dist_to_move ,robot.position.y,90,true);
		chThdSleepMilliseconds(5000);
		goals[i].position.x = robot.position.x;
		goals[i].position.y = robot.max.y;
		i++;
		light_rgb_led(OFF);
	}


  //SCAN FOURTH WALL
  if(i == THIRD_GOAL){
	 	identified_goal = false;
	 	move_robot(DIST_CAMERA_MEASURE,(robot.max.y - SECURE_DIST),180,true);
	 	chThdSleepMilliseconds(1000);
	 	while(!identified_goal){
	 		chThdSleepMilliseconds(500);
	 		if(get_color_detected() != WHITE){
	 			identified_goal=true;
	 		}
	 		else if((robot.position.y - STEP_POSITION) <= SECURE_DIST)
	 			break;
	 		else
	 			move_robot(DIST_CAMERA_MEASURE, (robot.position.y - STEP_POSITION),180,false);
	 		chThdSleepMilliseconds(1000);
	 	}
	 	goals[i].color = get_color_detected();
	 	light_rgb_led(goals[i].color);
	 	color_to_analyse = chose_color_to_analyse(goals[i].color);
		while(!(get_line_width(color_to_analyse) > 0)){
			move_robot(robot.position.x + STEP_POSITION, robot.position.y,180,true);
			chThdSleepMilliseconds(500);
		}
		line_position = get_line_position();
		//robot.position.y = retrieve_TOF_measure(10);
		dist_to_move = robot.position.y*TAN_CAMERA_APERTURE*(line_position-320)/320;
		move_robot(robot.position.x ,robot.position.y+dist_to_move,180,true);
		chThdSleepMilliseconds(5000);
		goals[i].position.x = 0;
		goals[i].position.y = robot.position.y;
		i++;
		light_rgb_led(OFF);
  }


}

void determine_world_dimension(uint8_t nbr_of_measures){
	adjust_orientation();
	robot.max.x =  retrieve_TOF_measure(nbr_of_measures) + DIAMETER_ROBOT/2;
	robot.position.x = robot.max.x;
	robot.orientation = 180;
	robot.orientation = rotate_angle(90,robot.orientation);
	robot.max.y = retrieve_TOF_measure(nbr_of_measures)+ DIAMETER_ROBOT/2;
	robot.position.y = robot.max.y;
	robot.orientation = rotate_angle(90,robot.orientation);
	robot.max.x += retrieve_TOF_measure(nbr_of_measures)+ DIAMETER_ROBOT/2;
	robot.orientation = rotate_angle(90,robot.orientation);
	robot.max.y += retrieve_TOF_measure(nbr_of_measures)+ DIAMETER_ROBOT/2;
	robot.orientation = rotate_angle(90,robot.orientation);
	move_robot(robot.position.x,robot.position.y,robot.orientation,false);
	move_robot((robot.max.x/2), (robot.max.y/2),0,false);
	chprintf((BaseSequentialStream *)&SD3, "dimensioni x: %d   e y : %d	\r\n\n",robot.max.x,robot.max.y);


}








	/***************************EXTERNAL FUNCTIONS************************************/
	void init_identify_world(void){
		chThdSleepMilliseconds(5000);
		robot.orientation = 0;
		determine_world_dimension(3);
		move_robot(robot.position.x,robot.position.y,robot.orientation,false);
		determine_world_dimension(20);
		//DETERMINARE LE PORTE
		chThdSleepMilliseconds(2000);
		attribute_goal();
		for(int i=0; i<3;i++){
			switch(goals[i].color){
			case RED:
				chprintf((BaseSequentialStream *)&SD3, "Porta %d colore: ROSSO	\r\n\n",i);
				chprintf((BaseSequentialStream *)&SD3, "posizione porta: x:%d		y:%d	\r\n\n",goals[i].position.x,goals[i].position.y);
				break;
			case GREEN:
				chprintf((BaseSequentialStream *)&SD3, "Porta %d colore: VERDE	\r\n\n",i);
				chprintf((BaseSequentialStream *)&SD3, "posizione porta: x:%d		y:%d	\r\n\n",goals[i].position.x,goals[i].position.y);
				break;
			case BLUE:
				chprintf((BaseSequentialStream *)&SD3, "Porta %d colore: BLU	\r\n\n",i);
				chprintf((BaseSequentialStream *)&SD3, "posizione porta: x:%d		y:%d	\r\n\n",goals[i].position.x,goals[i].position.y);
				break;
			case WHITE:
				chprintf((BaseSequentialStream *)&SD3, "Porta %d colore: BIANCO	\r\n\n",i);
				break;
			case COLOR_NOT_ATTRIBUTED:
				chprintf((BaseSequentialStream *)&SD3, "Porta %d colore: NON ATTRIBUITO	\r\n\n",i);
			}
		}



	}


