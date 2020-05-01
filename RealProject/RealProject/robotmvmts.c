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
#include <robotmvmts.h>
#include <main.h>
#include "sensors/VL53L0X/VL53L0X.h"
#include "process_image.h"
#include "audio_processing.h"
#include "leds.h"
#include <audio/microphone.h>
<<<<<<< HEAD
#include "audio/play_melody.h"
#include "audio/play_sound_file.h"
#include "audio/audio_thread.h"
=======
>>>>>>> 8624660db6ce97125d5ca04f847a51ed0c81d01e

/***************************INTERNAL FUNCTIONS************************************/
static Robot robot;
static Goal goals[NB_GOALS];
static Position desired_position;
static float desired_goal_orientation;
<<<<<<< HEAD
static bool stop_camera = false;

//input:	number of times that the TOF repeats the measure (nbr_of_measures)
//output: the average value of the different TOF measures [mm](dist)
//purpose: measure the distance robot-object in mm
=======


>>>>>>> 8624660db6ce97125d5ca04f847a51ed0c81d01e
uint16_t retrieve_TOF_measure(uint8_t nbr_of_measures){
	uint16_t dist = 0;
	for(int i =0; i<nbr_of_measures;i++){
		chThdSleepMilliseconds(150);
		dist+=VL53L0X_get_dist_mm();
	}
	dist = dist/nbr_of_measures;
	return dist;
}

<<<<<<< HEAD
//input:	-
//output: -
//purpose: uses trigonometry to re-adjust the robot's orientation so that it is perpendicular to the wall it's facing (the wall that the TOF sees)
void adjust_orientation(void){
	uint16_t dist1=0,dist2=0;
	float angle = 0;
	dist1 = retrieve_TOF_measure(QUICK_MEASURE);
	rotate_angle(ADJ_ORIENT_ANGLE,robot.orientation);
	dist2 = retrieve_TOF_measure(QUICK_MEASURE);
	// we calculate the denominator so that we don't do it repeatidly during the function (less calculation time in total)
	float denominator = sqrtf((dist1*dist1)-(2*dist1*dist2*COS20CTE)+dist2*dist2);
	angle = atan2f(((dist1-(dist2*COS20CTE))/denominator), (-dist2*SIN20CTE)/denominator) *RADIANT_TO_DEGREES;
	if(angle > 0)
		rotate_angle(-ADJ_ORIENT_ANGLE-angle+180,robot.orientation);
	else
		rotate_angle(-ADJ_ORIENT_ANGLE-angle-180,robot.orientation);

}

//input:	 desired_x,desired_y,desired_orientation: describe the position & orientation the robot should have at the end of the function
//remeasure_pos: if it's true, the robot remeasures his current position in the world using the TOF sensor
//step_mvmt : if it's true, the robot moves only a short distance towards the desired position (this is used in the thread)
//output: returns nothing but updates the position and the orientation of the robot at the end of the function
//purpose: manages all movements of the robot (translation & rotations)
void move_robot(uint16_t desired_x, uint16_t desired_y,float desired_orientation,bool remeasure_pos, bool step_mvmt){
	if(remeasure_pos){
		move_robot(robot.position.x,robot.position.y,FACE_WALL4,false,false);
		adjust_orientation();
		robot.orientation = 180;
		robot.position.x = retrieve_TOF_measure(NORMAL_MEASURE)+DIAMETER_ROBOT/2;
		move_robot(robot.position.x,robot.position.y,FACE_WALL1,false,false);
		robot.position.y = retrieve_TOF_measure(NORMAL_MEASURE)+DIAMETER_ROBOT/2;
=======
void adjust_orientation(void){
	uint16_t dist1=0,dist2=0;
	float angle = 0;
	dist1 = retrieve_TOF_measure(5);
	rotate_angle(20,robot.orientation);
	dist2 = retrieve_TOF_measure(5);
	// we calculate the denominator so that we don't do it repeatidly during the function (less calculation time in total)
	float denominator = sqrtf((dist1*dist1)-(2*dist1*dist2*COS20CTE)+dist2*dist2);
	angle = atan2f(((dist1-(dist2*COS20CTE))/denominator), (-dist2*SIN20CTE)/denominator) *180/M_PI;
	if(angle > 0)
		rotate_angle(-20-angle+180,robot.orientation);
	else
		rotate_angle(-20-angle-180,robot.orientation);
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

void move_robot(uint16_t desired_x, uint16_t desired_y,float desired_orientation,bool remeasure_pos, bool step_mvmt){
	//chprintf((BaseSequentialStream *)&SD3, "desired x: %d  desired y: %d	\r\n\n", desired_x, desired_y);
	if(remeasure_pos){
		move_robot(robot.position.x,robot.position.y,180,false,false);
		adjust_orientation();
		robot.orientation = 180;
		robot.position.x = retrieve_TOF_measure(10)+DIAMETER_ROBOT/2;
		move_robot(robot.position.x,robot.position.y,-90,false,false);
		robot.position.y = retrieve_TOF_measure(10)+DIAMETER_ROBOT/2;
>>>>>>> 8624660db6ce97125d5ca04f847a51ed0c81d01e
	}

	int16_t delta_x = desired_x-robot.position.x;
	int16_t delta_y = desired_y-robot.position.y;

	float distance = 0;
	float angle = 0;
<<<<<<< HEAD
	if(delta_x == 0 && delta_y == 0); // @suppress("Suspicious semicolon")
	//atanf returns an angle between -pi/2 -> pi/2 so we need rechange the angle so that it fits to our application
	else{
		if(delta_x != 0)
			angle= atanf((float)((float)delta_y/(float)delta_x))*RADIANT_TO_DEGREES;
		if((delta_x > 0 && delta_y > 0)||(delta_x > 0 && delta_y < 0))
			robot.orientation = rotate_angle((angle-robot.orientation),robot.orientation);
		else if((delta_x < 0 && delta_y > 0)||(delta_x < 0 && delta_y < 0))
=======
	//static int i = 0;
	if(delta_x == 0 && delta_y == 0); // @suppress("Suspicious semicolon")
	else{
		if(delta_x != 0)
			angle= atanf((float)((float)delta_y/(float)delta_x))*180/M_PI;

		//chprintf((BaseSequentialStream *)&SD3, "angle: %f	\r\n\n", angle);
		//chprintf((BaseSequentialStream *)&SD3, "angle-robotorientation: %f	\r\n\n", (angle-robot.orientation));
		if((delta_x > 0 && delta_y > 0)||(delta_x > 0 && delta_y < 0))
			robot.orientation = rotate_angle((angle-robot.orientation),robot.orientation);
		else if((delta_x < 0 && delta_y >0)||(delta_x < 0 && delta_y < 0))
>>>>>>> 8624660db6ce97125d5ca04f847a51ed0c81d01e
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
<<<<<<< HEAD
		distance = (float)(sqrtf(((delta_x*delta_x)+(delta_y*delta_y))));
		if(step_mvmt && distance > STEP_POSITION)
			distance = STEP_POSITION;
		move(distance);
	}
=======

		//chprintf((BaseSequentialStream *)&SD3, "distance : %f \r\n\n",distance);
		distance = (float)(sqrtf(((delta_x*delta_x)+(delta_y*delta_y))));
		if(step_mvmt && distance > STEP_POSITION)
			distance = STEP_POSITION;
		//chprintf((BaseSequentialStream *)&SD3, "distance: %f	\r\n\n", distance);
		//chprintf((BaseSequentialStream *)&SD3, " %d	\r\n\n", i);
		move(distance);
	}
	//chprintf((BaseSequentialStream *)&SD3, "angle: %f , deltax: %d,  deltay: %d	\r\n\n", angle,delta_x,delta_y);

	//chprintf((BaseSequentialStream *)&SD3, "IN MOVE ROBOT delta x = %d, delta y = %d, angle: %f	\r\n\n",delta_x,delta_y, angle);
	//chprintf((BaseSequentialStream *)&SD3, "INT MOVE ROBOT orientation = %f   x = %d    y = %d"  ,robot.orientation,robot.max.x,robot.max.y);
>>>>>>> 8624660db6ce97125d5ca04f847a51ed0c81d01e
	if(step_mvmt && distance == STEP_POSITION){
		robot.position.x += distance*cosf(robot.orientation*M_PI/180);
		robot.position.y += distance*sinf(robot.orientation*M_PI/180);
	}
	else{
		robot.position.x = desired_x;
		robot.position.y = desired_y;
<<<<<<< HEAD
		robot.orientation = rotate_angle(0,robot.orientation); //initilizazione(magari da togliere)
		robot.orientation = rotate_angle(desired_orientation-robot.orientation,robot.orientation);
	}
}


//input:	 color of goal that camera is facing
//output: returns an opposite color
//purpose: returns an opposite color to facilitate the analysis of the goal's position

=======
		robot.orientation = rotate_angle(0,robot.orientation); //initilizazione
		robot.orientation = rotate_angle(desired_orientation-robot.orientation,robot.orientation);
	}
}
	/***************************END INTERNAL FUNCTIONS************************************/
>>>>>>> 8624660db6ce97125d5ca04f847a51ed0c81d01e
uint8_t chose_color_to_analyse(uint8_t goal_color){
	switch(goal_color){
		case RED :
			return BLUE;
		default:
			return RED;
	}
}

<<<<<<< HEAD
//input:	 -
//output: -
//purpose: scan's all the walls and determines whether there's a goal or not on it. if it does, it memorizes it's color and position in the world
// the results are save in the struct array goals
void attribute_goal(void){
	uint8_t color_to_analyse;
	uint16_t line_position;
	int16_t dist_to_move;
	for(int i = FIRST_GOAL; i<=THIRD_GOAL ; i++)
=======
void attribute_goal(void){
	//AGGIUSTARE LE ORIENTAZIONI
	uint8_t color_to_analyse;
	uint16_t line_position;
	int16_t dist_to_move;
	for(int i = FIRST_GOAL; i<3 ; i++)
>>>>>>> 8624660db6ce97125d5ca04f847a51ed0c81d01e
		goals[i].color = COLOR_NOT_ATTRIBUTED;
	//SCAN FIRST WALL
	uint8_t i = FIRST_GOAL;
	bool identified_goal = false;
<<<<<<< HEAD
	move_robot(SECURE_DIST,DIST_CAMERA_MEASURE,FACE_WALL1,true,false);
=======
	move_robot(SECURE_DIST,DIST_CAMERA_MEASURE,-90,true,false);
	chThdSleepMilliseconds(1000);
>>>>>>> 8624660db6ce97125d5ca04f847a51ed0c81d01e
	while(!identified_goal){
		chThdSleepMilliseconds(500);
		if(get_color_detected() != WHITE)
			identified_goal=true;
		else if((robot.position.x + STEP_POSITION) >= (robot.max.x - SECURE_DIST))
			break;
		else
<<<<<<< HEAD
			move_robot(robot.position.x + STEP_POSITION, DIST_CAMERA_MEASURE,FACE_WALL1,false,false);
=======
			move_robot(robot.position.x + STEP_POSITION, DIST_CAMERA_MEASURE,-90,false,false);
		chThdSleepMilliseconds(1000);
>>>>>>> 8624660db6ce97125d5ca04f847a51ed0c81d01e
	}
	goals[i].color = get_color_detected();
	//CHECKING IF COLOR WAS ATTRIBUTED TO GOAL
	if(goals[i].color == WHITE)
		goals[i].color = COLOR_NOT_ATTRIBUTED;
	else{
<<<<<<< HEAD
		move_robot(robot.position.x + STEP_POSITION, DIST_CAMERA_MEASURE,FACE_WALL1,false,false);
		light_rgb_led(goals[i].color);
		color_to_analyse = chose_color_to_analyse(goals[i].color);
		while(!(get_line_width(color_to_analyse) > 0 || get_line_width(color_to_analyse) > 0)){
			move_robot(robot.position.x, robot.position.y + STEP_POSITION,FACE_WALL1,true,false);
			chThdSleepMilliseconds(500);
		}
		line_position = get_line_position();
		dist_to_move = robot.position.y*TAN_CAMERA_APERTURE*(line_position-CENTER_PIXEL_IMG)/CENTER_PIXEL_IMG;
		move_robot(robot.position.x - dist_to_move,robot.position.y,FACE_WALL1,true,false);
		goals[i].position.x = robot.position.x ;
		goals[i].position.y = DIAMETER_ROBOT;
=======
		move_robot(robot.position.x + 20, DIST_CAMERA_MEASURE,-90,false,false);
		light_rgb_led(goals[i].color);
		color_to_analyse = chose_color_to_analyse(goals[i].color);
		while(!(get_line_width(color_to_analyse) > 0 || get_line_width(color_to_analyse) > 0)){
			move_robot(robot.position.x, robot.position.y + STEP_POSITION,-90,true,false);
			chThdSleepMilliseconds(500);
		}
		line_position = get_line_position();
		//robot.position.y = retrieve_TOF_measure(10);
		dist_to_move = robot.position.y*TAN_CAMERA_APERTURE*(line_position-320)/320;
		move_robot(robot.position.x - dist_to_move,robot.position.y,-90,true,false);
		chThdSleepMilliseconds(5000);
		goals[i].position.x = robot.position.x ;
		goals[i].position.y = DIAMETER_ROBOT / 2;
>>>>>>> 8624660db6ce97125d5ca04f847a51ed0c81d01e
		goals[i].orientation = robot.orientation;
		i++;
		light_rgb_led(OFF);
	}



    	//SCAN SECOND WALL
    	identified_goal = false;
<<<<<<< HEAD
    	move_robot(robot.max.x - DIST_CAMERA_MEASURE,SECURE_DIST,FACE_WALL2,true,false);
    	while(!identified_goal){
    		chThdSleepMilliseconds(500);
    		if(get_color_detected() != WHITE)
    			identified_goal=true;
    		else if((robot.position.y + STEP_POSITION) >= (robot.max.y - SECURE_DIST))
    			break;
    		else
    			move_robot(robot.max.x - DIST_CAMERA_MEASURE, robot.position.y + STEP_POSITION,FACE_WALL2,false,false);
=======
    	move_robot(robot.max.x - DIST_CAMERA_MEASURE,SECURE_DIST,0,true,false);
    	chThdSleepMilliseconds(1000);
    	while(!identified_goal){
    		chThdSleepMilliseconds(500);
    		if(get_color_detected() != WHITE){
    			identified_goal=true;
    		}
    		else if((robot.position.y + STEP_POSITION) >= (robot.max.y - SECURE_DIST))
    			break;
    		else
    			move_robot(robot.max.x - DIST_CAMERA_MEASURE, robot.position.y + STEP_POSITION,0,false,false);
    		chThdSleepMilliseconds(1000);
>>>>>>> 8624660db6ce97125d5ca04f847a51ed0c81d01e
    	}
    	goals[i].color = get_color_detected();

    	//CHECKING IF COLOR WAS ATTRIBUTED TO GOAL
    	if(goals[i].color == WHITE)
    		goals[i].color = COLOR_NOT_ATTRIBUTED;
    	else{
<<<<<<< HEAD
    		move_robot(robot.max.x - DIST_CAMERA_MEASURE, robot.position.y + STEP_POSITION,FACE_WALL2,false,false);
    		light_rgb_led(goals[i].color);
    		color_to_analyse = chose_color_to_analyse(goals[i].color);
    		while(!(get_line_width(color_to_analyse) > 0 || get_line_width(color_to_analyse) > 0)){
    			move_robot(robot.position.x - STEP_POSITION, robot.position.y,FACE_WALL2,true,false);
=======
    		move_robot(robot.max.x - DIST_CAMERA_MEASURE, robot.position.y + 20,0,false,false);
    		light_rgb_led(goals[i].color);
    		color_to_analyse = chose_color_to_analyse(goals[i].color);
    		while(!(get_line_width(color_to_analyse) > 0 || get_line_width(color_to_analyse) > 0)){
    			move_robot(robot.position.x - STEP_POSITION, robot.position.y,0,true,false);
>>>>>>> 8624660db6ce97125d5ca04f847a51ed0c81d01e
    			adjust_orientation();
    			chThdSleepMilliseconds(500);
    		}
    		line_position = get_line_position();
    		//robot.position.x = robot.max.x-retrieve_TOF_measure(10);
<<<<<<< HEAD
    		dist_to_move = (robot.max.x-robot.position.x)*TAN_CAMERA_APERTURE*(line_position-CENTER_PIXEL_IMG)/CENTER_PIXEL_IMG;
    		move_robot(robot.position.x ,robot.position.y-dist_to_move,FACE_WALL2,true,false);
    		goals[i].position.x = robot.max.x - DIAMETER_ROBOT;
=======
    		dist_to_move = (robot.max.x-robot.position.x)*TAN_CAMERA_APERTURE*(line_position-320)/320;
    		move_robot(robot.position.x ,robot.position.y-dist_to_move,0,true,false);
    		chThdSleepMilliseconds(5000);
    		goals[i].position.x = robot.max.x - DIAMETER_ROBOT / 2;
>>>>>>> 8624660db6ce97125d5ca04f847a51ed0c81d01e
    		goals[i].position.y = robot.position.y;
    		goals[i].orientation = robot.orientation;
    		i++;
    		light_rgb_led(OFF);
    	}

    //SCAN THIRD GOAL
 	identified_goal = false;
<<<<<<< HEAD
 	move_robot((robot.max.x - SECURE_DIST),(robot.max.y - DIST_CAMERA_MEASURE),FACE_WALL3,true,false);
 	while(!identified_goal){
 		chThdSleepMilliseconds(500);
 		if(get_color_detected() != WHITE)
 			identified_goal=true;
 		else if((robot.position.x - STEP_POSITION) <= SECURE_DIST)
 			break;
 		else
 			move_robot(robot.position.x-STEP_POSITION,(robot.max.y - DIST_CAMERA_MEASURE),FACE_WALL3,false,false);
=======
 	move_robot((robot.max.x - SECURE_DIST),(robot.max.y - DIST_CAMERA_MEASURE),90,true,false);
 	chThdSleepMilliseconds(1000);
 	while(!identified_goal){
 		chThdSleepMilliseconds(500);
 		if(get_color_detected() != WHITE){
 			identified_goal=true;
 		}
 		else if((robot.position.x - STEP_POSITION) <= SECURE_DIST)
 			break;
 		else
 			move_robot(robot.position.x-STEP_POSITION,(robot.max.y - DIST_CAMERA_MEASURE),90,false,false);
 		chThdSleepMilliseconds(1000);
>>>>>>> 8624660db6ce97125d5ca04f847a51ed0c81d01e
 	}
 	goals[i].color = get_color_detected();

 	//CHECKING IF COLOR WAS ATTRIBUTED TO GOAL
 	if(goals[i].color == WHITE)
 		goals[i].color = COLOR_NOT_ATTRIBUTED;
	else{
<<<<<<< HEAD
		move_robot(robot.position.x-STEP_POSITION,(robot.max.y - DIST_CAMERA_MEASURE),FACE_WALL3,false,false);
		light_rgb_led(goals[i].color);
		color_to_analyse = chose_color_to_analyse(goals[i].color);
		while(!(get_line_width(color_to_analyse) > 0 || get_line_width(color_to_analyse) > 0)){
			move_robot(robot.position.x, robot.position.y-STEP_POSITION,FACE_WALL3,true,false);
=======
		move_robot(robot.position.x-20,(robot.max.y - DIST_CAMERA_MEASURE),90,false,false);
		light_rgb_led(goals[i].color);
		color_to_analyse = chose_color_to_analyse(goals[i].color);
		while(!(get_line_width(color_to_analyse) > 0 || get_line_width(color_to_analyse) > 0)){
			move_robot(robot.position.x, robot.position.y-STEP_POSITION,90,true,false);
>>>>>>> 8624660db6ce97125d5ca04f847a51ed0c81d01e
			chThdSleepMilliseconds(500);
		}
		line_position = get_line_position();
		//robot.position.y = robot.max.y - retrieve_TOF_measure(10);
<<<<<<< HEAD
		dist_to_move = (robot.max.y-robot.position.y)*TAN_CAMERA_APERTURE*(line_position-CENTER_PIXEL_IMG)/CENTER_PIXEL_IMG;
		move_robot(robot.position.x + dist_to_move ,robot.position.y,FACE_WALL3,true,false);
		goals[i].position.x = robot.position.x;
		goals[i].position.y = robot.max.y - DIAMETER_ROBOT;
=======
		dist_to_move = (robot.max.y-robot.position.y)*TAN_CAMERA_APERTURE*(line_position-320)/320;
		move_robot(robot.position.x + dist_to_move ,robot.position.y,90,true,false);
		chThdSleepMilliseconds(5000);
		goals[i].position.x = robot.position.x;
		goals[i].position.y = robot.max.y - DIAMETER_ROBOT / 2;
>>>>>>> 8624660db6ce97125d5ca04f847a51ed0c81d01e
		goals[i].orientation = robot.orientation;
		i++;
		light_rgb_led(OFF);
	}


  //SCAN FOURTH WALL
  if(i == THIRD_GOAL){
	 	identified_goal = false;
<<<<<<< HEAD
	 	move_robot(DIST_CAMERA_MEASURE,(robot.max.y - SECURE_DIST),FACE_WALL4,true,false);
=======
	 	move_robot(DIST_CAMERA_MEASURE,(robot.max.y - SECURE_DIST),180,true,false);
	 	chThdSleepMilliseconds(1000);
>>>>>>> 8624660db6ce97125d5ca04f847a51ed0c81d01e
	 	while(!identified_goal){
	 		chThdSleepMilliseconds(500);
	 		if(get_color_detected() != WHITE){
	 			identified_goal=true;
	 		}
	 		else if((robot.position.y - STEP_POSITION) <= SECURE_DIST)
	 			break;
	 		else
<<<<<<< HEAD
	 			move_robot(DIST_CAMERA_MEASURE, (robot.position.y - STEP_POSITION),FACE_WALL4,false,false);
=======
	 			move_robot(DIST_CAMERA_MEASURE, (robot.position.y - STEP_POSITION),180,false,false);
	 		chThdSleepMilliseconds(1000);
>>>>>>> 8624660db6ce97125d5ca04f847a51ed0c81d01e
	 	}
	 	goals[i].color = get_color_detected();
	 	if(goals[i].color == WHITE)
	 		goals[i].color = COLOR_NOT_ATTRIBUTED;
	 	else{
<<<<<<< HEAD
	 		move_robot(DIST_CAMERA_MEASURE,(robot.position.y - STEP_POSITION),FACE_WALL4,true,false);
			light_rgb_led(goals[i].color);
			color_to_analyse = chose_color_to_analyse(goals[i].color);
			while(!(get_line_width(color_to_analyse) > 0 || get_line_width(color_to_analyse) > 0)){
				move_robot(robot.position.x + STEP_POSITION, robot.position.y,FACE_WALL4,true,false);
				chThdSleepMilliseconds(500);
			}
			line_position = get_line_position();
			dist_to_move = robot.position.y*TAN_CAMERA_APERTURE*(line_position-CENTER_PIXEL_IMG)/CENTER_PIXEL_IMG;
			move_robot(robot.position.x ,robot.position.y+dist_to_move,FACE_WALL4,true,false);
			goals[i].position.x = DIAMETER_ROBOT;
=======
	 		move_robot(DIST_CAMERA_MEASURE,(robot.max.y - 20),180,true,false);
			light_rgb_led(goals[i].color);
			color_to_analyse = chose_color_to_analyse(goals[i].color);
			while(!(get_line_width(color_to_analyse) > 0 || get_line_width(color_to_analyse) > 0)){
				move_robot(robot.position.x + STEP_POSITION, robot.position.y,180,true,false);
				chThdSleepMilliseconds(500);
			}
			line_position = get_line_position();
			//robot.position.y = retrieve_TOF_measure(10);
			dist_to_move = robot.position.y*TAN_CAMERA_APERTURE*(line_position-320)/320;
			move_robot(robot.position.x ,robot.position.y+dist_to_move,180,true,false);
			chThdSleepMilliseconds(5000);
			goals[i].position.x = DIAMETER_ROBOT / 2;
>>>>>>> 8624660db6ce97125d5ca04f847a51ed0c81d01e
			goals[i].position.y = robot.position.y;
			goals[i].orientation = robot.orientation;
			i++;
			light_rgb_led(OFF);
	 	}


  	  }
}

<<<<<<< HEAD
//input:	 number of times that the TOF should take the measure
//output: -
//purpose: Determination of world dimension (x_max,y_max) and of the robot's position (robot.position.x,robot.position.y)

void determine_world_dimension(uint8_t nbr_of_measures){
	adjust_orientation();
	robot.max.x =  retrieve_TOF_measure(nbr_of_measures) + DIAMETER_ROBOT/2;
	robot.position.x = robot.max.x;
	robot.orientation = FACE_WALL4;
=======
void determine_world_dimension(uint8_t nbr_of_measures){
	adjust_orientation();
	chThdSleepMilliseconds(5000);
	robot.max.x =  retrieve_TOF_measure(nbr_of_measures) + DIAMETER_ROBOT/2;
	robot.position.x = robot.max.x;
	robot.orientation = 180;
>>>>>>> 8624660db6ce97125d5ca04f847a51ed0c81d01e
	robot.orientation = rotate_angle(90,robot.orientation);
	robot.max.y = retrieve_TOF_measure(nbr_of_measures)+ DIAMETER_ROBOT/2;
	robot.position.y = robot.max.y;
	robot.orientation = rotate_angle(90,robot.orientation);
	robot.max.x += retrieve_TOF_measure(nbr_of_measures)+ DIAMETER_ROBOT/2;
	robot.orientation = rotate_angle(90,robot.orientation);
	robot.max.y += retrieve_TOF_measure(nbr_of_measures)+ DIAMETER_ROBOT/2;
	robot.orientation = rotate_angle(90,robot.orientation);
<<<<<<< HEAD
	move_robot(robot.position.x,robot.position.y,robot.orientation,false,false); //da vedere se possiamo togliere
	move_robot((robot.max.x/2), (robot.max.y/2),0,false,false);
}

//input:	 -
//output: -
//purpose: depending on the frequency heard by the robot (in get_goal_destination), it assigns to the robot the goal to reach

void manage_desired_goal(void){
	static int8_t prev_goal = -1;
	int8_t des_goal = get_goal_destination();
		switch(des_goal){
			case RED:
				if(prev_goal != des_goal){
					playMelody(CALIFORNICATION, ML_FORCE_CHANGE,NULL);
					prev_goal = RED;
					move_robot(desired_position.x, desired_position.y,desired_goal_orientation,true, true);
				}
				else
					playMelody(CALIFORNICATION, ML_SIMPLE_PLAY,NULL);
				desired_position = goals[RED].position;
				desired_goal_orientation = goals[RED].orientation;
				break;
			case BLUE:
				if(prev_goal != des_goal){
					playMelody(IM_BLUE, ML_FORCE_CHANGE,NULL);
					prev_goal = BLUE;
					move_robot(desired_position.x, desired_position.y,desired_goal_orientation,true, true);
				}
				else
					playMelody(IM_BLUE, ML_SIMPLE_PLAY,NULL);
				desired_position = goals[BLUE].position;
				desired_goal_orientation = goals[BLUE].orientation;
				break;
			case GREEN:
				if(prev_goal != des_goal){
					playMelody(BROKEN_DREAMS, ML_FORCE_CHANGE,NULL);
					prev_goal = GREEN;
					move_robot(desired_position.x, desired_position.y,desired_goal_orientation,true, true);
				}
				else
					playMelody(BROKEN_DREAMS, ML_SIMPLE_PLAY,NULL);
				desired_position = goals[GREEN].position;
				desired_goal_orientation = goals[GREEN].orientation;
				break;
			default:
				switch(prev_goal){
					case RED:
						playMelody(CALIFORNICATION, ML_SIMPLE_PLAY,NULL);
						break;
					case BLUE:
						playMelody(IM_BLUE,ML_SIMPLE_PLAY,NULL);
						break;
					case GREEN:
						playMelody(BROKEN_DREAMS,ML_SIMPLE_PLAY,NULL);
						break;
				}
				break;
	}
}

//input:	 -
//output: -
//purpose: orders the  goals array in the following sequence: FIRST GOAL is RED ; SECOND GOAL is BLUE ; THIRD GOAL is GREEN
=======
	move_robot(robot.position.x,robot.position.y,robot.orientation,false,false);
	move_robot((robot.max.x/2), (robot.max.y/2),0,false,false);
	chprintf((BaseSequentialStream *)&SD3, "dimensioni x: %d   e y : %d	\r\n\n",robot.max.x,robot.max.y);


}
void manage_desired_goal(void){
	int8_t des_goal = get_goal_destination();
	switch(des_goal){
		case RED:
			desired_position = goals[RED].position;
			desired_goal_orientation = goals[RED].orientation;
			break;
		case BLUE:
			desired_position = goals[BLUE].position;
			desired_goal_orientation = goals[BLUE].orientation;
			break;
		case GREEN:
			desired_position = goals[GREEN].position;
			desired_goal_orientation = goals[GREEN].orientation;
			break;
	}

}
>>>>>>> 8624660db6ce97125d5ca04f847a51ed0c81d01e

void order_goals(void){
	Goal tmp;
	for(uint8_t i = FIRST_GOAL; i < THIRD_GOAL ; i++){
		switch(goals[i].color){
			case RED:
				if(i != FIRST_GOAL){
					tmp = goals[FIRST_GOAL];
					goals[FIRST_GOAL] = goals[i];
					goals[i] = tmp;
<<<<<<< HEAD
=======
					chprintf((BaseSequentialStream *)&SD3, "ho ordinato il ROSSO	\r\n\n");
>>>>>>> 8624660db6ce97125d5ca04f847a51ed0c81d01e
				}
				break;
			case BLUE:
				if(i != SECOND_GOAL){
					tmp = goals[SECOND_GOAL];
					goals[SECOND_GOAL] = goals[i];
					goals[i] = tmp;
<<<<<<< HEAD
=======
					chprintf((BaseSequentialStream *)&SD3, "ho ordinato il BLU	\r\n\n");
>>>>>>> 8624660db6ce97125d5ca04f847a51ed0c81d01e
				}
				break;
			case GREEN:
				if(i != THIRD_GOAL){
					tmp = goals[THIRD_GOAL];
					goals[THIRD_GOAL] = goals[i];
					goals[i] = tmp;
<<<<<<< HEAD
=======
					chprintf((BaseSequentialStream *)&SD3, "ho ordinato il VERDE	\r\n\n");
>>>>>>> 8624660db6ce97125d5ca04f847a51ed0c81d01e
				}
				break;
		}
	}
}

<<<<<<< HEAD
//input:	 -
//output: -
//purpose: function of higher order that puts together all the functions to identify the world (max dimensions & goals color and position)

void init_identify_world(void){
		robot.orientation = 0;
		determine_world_dimension(PRECISE_MEASURE);
		move_robot(robot.position.x,robot.position.y,robot.orientation,false,false); //magari da tolgliere. verificare!!!!!
		attribute_goal();
		order_goals();
}
	/***************************END INTERNAL FUNCTIONS************************************/

/***************************THREAD************************************/

//
=======
void init_identify_world(void){
		//mi da il tempo di piazzarlo nell mondo
		chThdSleepMilliseconds(5000);
		robot.orientation = 0;
		determine_world_dimension(20);
		move_robot(robot.position.x,robot.position.y,robot.orientation,false,false);
		//determine_world_dimension(20);
		//DETERMINARE LE PORTE
		//chThdSleepMilliseconds(2000);
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
>>>>>>> 8624660db6ce97125d5ca04f847a51ed0c81d01e

static THD_WORKING_AREA(waGoingtoGoals, 1024);
static THD_FUNCTION(GoingtoGoals, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;
    init_identify_world();
<<<<<<< HEAD
=======
    order_goals();
    chprintf((BaseSequentialStream *)&SD3, "BANG BANG	\r\n\n");
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
>>>>>>> 8624660db6ce97125d5ca04f847a51ed0c81d01e
    mic_start(&processAudioData);
    desired_position.x = robot.position.x;
    desired_position.y = robot.position.y;
    desired_goal_orientation = robot.orientation;
<<<<<<< HEAD
    stop_camera = true;
    chThdSleepMilliseconds(500);
    while(1){
    		if(robot.position.x != desired_position.x || robot.position.y != desired_position.y)
    			move_robot(desired_position.x, desired_position.y,desired_goal_orientation,false, true);
    		else{
    			move_robot(robot.max.x/2, robot.max.y/2,desired_goal_orientation,false, false);
    		    desired_position.x = robot.position.x;
    		    desired_position.y = robot.position.y;
    		}
    		manage_desired_goal();
    		chThdSleepMilliseconds(10);
    }
}

	/***************************EXTERNAL FUNCTIONS************************************/
//input:	 -
//output: -
//purpose: initialises robotmvmts thread

void robotmvmts_start(void){
	chThdCreateStatic(waGoingtoGoals, sizeof(waGoingtoGoals), NORMALPRIO-1, GoingtoGoals, NULL);
}

bool retrieve_stop_camera(void){
	return stop_camera;
}


=======
    while(1){
    		if(robot.position.x != desired_position.x && robot.position.y != desired_position.y)
    			move_robot(desired_position.x, desired_position.y,desired_goal_orientation,false, true);
    		manage_desired_goal();
    }
}






	/***************************EXTERNAL FUNCTIONS************************************/

void robotmvmts_start(void){
	chThdCreateStatic(waGoingtoGoals, sizeof(waGoingtoGoals), NORMALPRIO, GoingtoGoals, NULL);
}

>>>>>>> 8624660db6ce97125d5ca04f847a51ed0c81d01e
