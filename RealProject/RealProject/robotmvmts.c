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
#include "audio/play_melody.h"
#include "audio/play_sound_file.h"
#include "audio/audio_thread.h"

/***************************INTERNAL FUNCTIONS************************************/
static Robot robot;
static Goal goals[NB_GOALS];
static Position desired_position;
static float desired_goal_orientation;

//input:	number of times that the TOF repeats the measure (nbr_of_measures)
//output: the average value of the different TOF measures [mm](dist)
//purpose: measure the distance robot-object in mm
uint16_t retrieve_TOF_measure(uint8_t nbr_of_measures){
	uint16_t dist = 0;
	for(int i =0; i<nbr_of_measures;i++){
		chThdSleepMilliseconds(150);
		dist+=VL53L0X_get_dist_mm();
	}
	dist = dist/nbr_of_measures;
	return dist;
}

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
	}

	int16_t delta_x = desired_x-robot.position.x;
	int16_t delta_y = desired_y-robot.position.y;

	float distance = 0;
	float angle = 0;
	if(delta_x == 0 && delta_y == 0); // @suppress("Suspicious semicolon")
	//atanf returns an angle between -pi/2 -> pi/2 so we need rechange the angle so that it fits to our application
	else{
		if(delta_x != 0)
			angle= atanf((float)((float)delta_y/(float)delta_x))*RADIANT_TO_DEGREES;
		if((delta_x > 0 && delta_y > 0)||(delta_x > 0 && delta_y < 0))
			robot.orientation = rotate_angle((angle-robot.orientation),robot.orientation);
		else if((delta_x < 0 && delta_y > 0)||(delta_x < 0 && delta_y < 0))
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
		distance = (float)(sqrtf(((delta_x*delta_x)+(delta_y*delta_y))));
		if(step_mvmt && distance > STEP_POSITION)
			distance = STEP_POSITION;
		move(distance);
	}
	if(step_mvmt && distance == STEP_POSITION){
		robot.position.x += distance*cosf(robot.orientation*M_PI/180);
		robot.position.y += distance*sinf(robot.orientation*M_PI/180);
	}
	else{
		robot.position.x = desired_x;
		robot.position.y = desired_y;
		robot.orientation = rotate_angle(0,robot.orientation); //initilizazione(magari da togliere)
		robot.orientation = rotate_angle(desired_orientation-robot.orientation,robot.orientation);
	}
}


//input:	 color of goal that camera is facing
//output: returns an opposite color
//purpose: returns an opposite color to facilitate the analysis of the goal's position

uint8_t chose_color_to_analyse(uint8_t goal_color){
	switch(goal_color){
		case RED :
			return BLUE;
		default:
			return RED;
	}
}

//input:	 -
//output: -
//purpose: scan's all the walls and determines whether there's a goal or not on it. if it does, it memorizes it's color and position in the world
// the results are save in the struct array goals
void attribute_goal(void){
	uint8_t color_to_analyse;
	uint16_t line_position;
	int16_t dist_to_move;
	uint8_t reorientation_cnt = 0;
	for(int i = FIRST_GOAL; i<=THIRD_GOAL ; i++)
		goals[i].color = COLOR_NOT_ATTRIBUTED;
	//SCAN FIRST WALL
	uint8_t i = FIRST_GOAL;
	bool identified_goal = false;
	move_robot(SECURE_DIST,DIST_CAMERA_MEASURE,FACE_WALL1,true,false);
	while(!identified_goal){
		chThdSleepMilliseconds(500);
		if(get_color_detected() != WHITE)
			identified_goal=true;
		else if((robot.position.x + STEP_POSITION) >= (robot.max.x - SECURE_DIST))
			break;
		else{
			if(reorientation_cnt < REOR_THRESHOLD)
				move_robot(robot.position.x + STEP_POSITION, DIST_CAMERA_MEASURE,FACE_WALL1,false,false);
			else{
				rotate_angle(-ADJ_ORIENT_ANGLE,robot.orientation);
				adjust_orientation();
				reorientation_cnt = 0;
			}
		}
		reorientation_cnt ++;
	}
	goals[i].color = get_color_detected();
	//CHECKING IF COLOR WAS ATTRIBUTED TO GOAL
	if(goals[i].color == WHITE)
		goals[i].color = COLOR_NOT_ATTRIBUTED;
	else{
		move_robot(robot.position.x + STEP_POSITION, DIST_CAMERA_MEASURE + 2* STEP_POSITION,FACE_WALL1,false,false);
		light_rgb_led(goals[i].color);
		color_to_analyse = chose_color_to_analyse(goals[i].color);
		while(!(get_line_width(color_to_analyse) > 0 || get_line_width(color_to_analyse) > 0)){
			if(robot.position.y > (robot.max.y - DIAMETER_ROBOT))
				break;
			move_robot(robot.position.x, robot.position.y + STEP_POSITION,FACE_WALL1,true,false);
			chThdSleepMilliseconds(500);
		}
		line_position = get_line_position();
		dist_to_move = robot.position.y*TAN_CAMERA_APERTURE*(line_position-CENTER_PIXEL_IMG)/CENTER_PIXEL_IMG;
		move_robot(robot.position.x - dist_to_move,robot.position.y,FACE_WALL1,true,false);
		goals[i].position.x = robot.position.x ;
		goals[i].position.y = DIAMETER_ROBOT;
		goals[i].orientation = robot.orientation;
		i++;
		light_rgb_led(OFF);
	}



    	//SCAN SECOND WALL
    	identified_goal = false;
    	reorientation_cnt = 0;
    	move_robot(robot.max.x/2,robot.max.y/2,robot.orientation,false,false);
    	move_robot(robot.max.x - DIST_CAMERA_MEASURE,SECURE_DIST,FACE_WALL2,true,false);
    	while(!identified_goal){
    		chThdSleepMilliseconds(500);
    		if(get_color_detected() != WHITE)
    			identified_goal=true;
    		else if((robot.position.y + STEP_POSITION) >= (robot.max.y - SECURE_DIST))
    			break;
    		else{
    			if(reorientation_cnt < REOR_THRESHOLD)
    				move_robot(robot.max.x - DIST_CAMERA_MEASURE, robot.position.y + STEP_POSITION,FACE_WALL2,false,false);
    			else{
    				rotate_angle(-ADJ_ORIENT_ANGLE,robot.orientation);
    				adjust_orientation();
    				reorientation_cnt = 0;
    			}
    		}
    		reorientation_cnt ++;
    	}
    	goals[i].color = get_color_detected();

    	//CHECKING IF COLOR WAS ATTRIBUTED TO GOAL
    	if(goals[i].color == WHITE)
    		goals[i].color = COLOR_NOT_ATTRIBUTED;
    	else{
    		move_robot(robot.max.x - DIST_CAMERA_MEASURE - 2* STEP_POSITION, robot.position.y + STEP_POSITION,FACE_WALL2,false,false);
    		light_rgb_led(goals[i].color);
    		color_to_analyse = chose_color_to_analyse(goals[i].color);
    		while(!(get_line_width(color_to_analyse) > 0 || get_line_width(color_to_analyse) > 0)){
    			if(robot.position.x <  DIAMETER_ROBOT)
    				break;
    			move_robot(robot.position.x - STEP_POSITION, robot.position.y,FACE_WALL2,true,false);
    			adjust_orientation();
    			chThdSleepMilliseconds(500);
    		}
    		line_position = get_line_position();
    		//robot.position.x = robot.max.x-retrieve_TOF_measure(10);
    		dist_to_move = (robot.max.x-robot.position.x)*TAN_CAMERA_APERTURE*(line_position-CENTER_PIXEL_IMG)/CENTER_PIXEL_IMG;
    		move_robot(robot.position.x ,robot.position.y-dist_to_move,FACE_WALL2,true,false);
    		goals[i].position.x = robot.max.x - DIAMETER_ROBOT;
    		goals[i].position.y = robot.position.y;
    		goals[i].orientation = robot.orientation;
    		i++;
    		light_rgb_led(OFF);
    	}

    //SCAN THIRD GOAL
    	reorientation_cnt = 0;
 	identified_goal = false;
	move_robot(robot.max.x/2,robot.max.y/2,robot.orientation,false,false);
 	move_robot((robot.max.x - SECURE_DIST),(robot.max.y - DIST_CAMERA_MEASURE),FACE_WALL3,true,false);
 	while(!identified_goal){
 		chThdSleepMilliseconds(500);
 		if(get_color_detected() != WHITE)
 			identified_goal=true;
 		else if((robot.position.x - STEP_POSITION) <= SECURE_DIST)
 			break;
 		else{
			if(reorientation_cnt < REOR_THRESHOLD)
				move_robot(robot.position.x-STEP_POSITION,(robot.max.y - DIST_CAMERA_MEASURE),FACE_WALL3,false,false);
			else{
				rotate_angle(-ADJ_ORIENT_ANGLE,robot.orientation);
				adjust_orientation();
				reorientation_cnt = 0;
			}
 		}
		reorientation_cnt ++;
 	}
 	goals[i].color = get_color_detected();

 	//CHECKING IF COLOR WAS ATTRIBUTED TO GOAL
 	if(goals[i].color == WHITE)
 		goals[i].color = COLOR_NOT_ATTRIBUTED;
	else{
		move_robot(robot.position.x-STEP_POSITION,(robot.max.y - DIST_CAMERA_MEASURE - 2* STEP_POSITION),FACE_WALL3,false,false);
		light_rgb_led(goals[i].color);
		color_to_analyse = chose_color_to_analyse(goals[i].color);
		while(!(get_line_width(color_to_analyse) > 0 || get_line_width(color_to_analyse) > 0)){
			if(robot.position.y < DIAMETER_ROBOT)
				break;
			move_robot(robot.position.x, robot.position.y-STEP_POSITION,FACE_WALL3,true,false);
			chThdSleepMilliseconds(500);
		}
		line_position = get_line_position();
		//robot.position.y = robot.max.y - retrieve_TOF_measure(10);
		dist_to_move = (robot.max.y-robot.position.y)*TAN_CAMERA_APERTURE*(line_position-CENTER_PIXEL_IMG)/CENTER_PIXEL_IMG;
		move_robot(robot.position.x + dist_to_move ,robot.position.y,FACE_WALL3,true,false);
		goals[i].position.x = robot.position.x;
		goals[i].position.y = robot.max.y - DIAMETER_ROBOT;
		goals[i].orientation = robot.orientation;
		i++;
		light_rgb_led(OFF);
	}


  //SCAN FOURTH WALL
  if(i == THIRD_GOAL){
	  	reorientation_cnt = 0;
	  	identified_goal = false;
		move_robot(robot.max.x/2,robot.max.y/2,robot.orientation,false,false);
	 	move_robot(DIST_CAMERA_MEASURE,(robot.max.y - SECURE_DIST),FACE_WALL4,true,false);
	 	while(!identified_goal){
	 		chThdSleepMilliseconds(500);
	 		if(get_color_detected() != WHITE){
	 			identified_goal=true;
	 		}
	 		else if((robot.position.y - STEP_POSITION) <= SECURE_DIST)
	 			break;
	 		else{
				if(reorientation_cnt < REOR_THRESHOLD)
					move_robot(DIST_CAMERA_MEASURE, (robot.position.y - STEP_POSITION),FACE_WALL4,false,false);
				else{
					rotate_angle(-ADJ_ORIENT_ANGLE,robot.orientation);
					adjust_orientation();
					reorientation_cnt = 0;
				}
	 		}
			reorientation_cnt ++;
	 	}
	 	goals[i].color = get_color_detected();
	 	if(goals[i].color == WHITE)
	 		goals[i].color = COLOR_NOT_ATTRIBUTED;
	 	else{
	 		move_robot(DIST_CAMERA_MEASURE + 2* STEP_POSITION,(robot.position.y - STEP_POSITION),FACE_WALL4,false,false);
			light_rgb_led(goals[i].color);
			color_to_analyse = chose_color_to_analyse(goals[i].color);
			while(!(get_line_width(color_to_analyse) > 0 || get_line_width(color_to_analyse) > 0)){
				if(robot.position.x > (robot.max.x - DIAMETER_ROBOT))
					break;
				move_robot(robot.position.x + STEP_POSITION, robot.position.y,FACE_WALL4,true,false);
				chThdSleepMilliseconds(500);
			}
			line_position = get_line_position();
			dist_to_move = robot.position.y*TAN_CAMERA_APERTURE*(line_position-CENTER_PIXEL_IMG)/CENTER_PIXEL_IMG;
			move_robot(robot.position.x ,robot.position.y+dist_to_move,FACE_WALL4,true,false);
			goals[i].position.x = DIAMETER_ROBOT;
			goals[i].position.y = robot.position.y;
			goals[i].orientation = robot.orientation;
			i++;
			light_rgb_led(OFF);
	 	}


  	  }
}

//input:	 number of times that the TOF should take the measure
//output: -
//purpose: Determination of world dimension (x_max,y_max) and of the robot's position (robot.position.x,robot.position.y)

void determine_world_dimension(uint8_t nbr_of_measures){
	adjust_orientation();
	robot.max.x =  retrieve_TOF_measure(nbr_of_measures) + DIAMETER_ROBOT/2;
	robot.position.x = robot.max.x;
	robot.orientation = FACE_WALL4;
	robot.orientation = rotate_angle(90,robot.orientation);
	robot.max.y = retrieve_TOF_measure(nbr_of_measures)+ DIAMETER_ROBOT/2;
	robot.position.y = robot.max.y;
	robot.orientation = rotate_angle(90,robot.orientation);
	robot.max.x += retrieve_TOF_measure(nbr_of_measures)+ DIAMETER_ROBOT/2;
	robot.orientation = rotate_angle(90,robot.orientation);
	robot.max.y += retrieve_TOF_measure(nbr_of_measures)+ DIAMETER_ROBOT/2;
	robot.orientation = rotate_angle(90,robot.orientation);
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

void order_goals(void){
	Goal tmp;
	for(uint8_t i = FIRST_GOAL; i <= THIRD_GOAL ; i++){
		switch(goals[i].color){
			case RED:
				if(i != FIRST_GOAL){
					tmp = goals[FIRST_GOAL];
					goals[FIRST_GOAL] = goals[i];
					goals[i] = tmp;
				}
				break;
			case BLUE:
				if(i != SECOND_GOAL){
					tmp = goals[SECOND_GOAL];
					goals[SECOND_GOAL] = goals[i];
					goals[i] = tmp;
				}
				break;
			case GREEN:
				if(i != THIRD_GOAL){
					tmp = goals[THIRD_GOAL];
					goals[THIRD_GOAL] = goals[i];
					goals[i] = tmp;
				}
				break;
		}
	}
}

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

static THD_WORKING_AREA(waGoingtoGoals, 1024);
static THD_FUNCTION(GoingtoGoals, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;
    init_identify_world();
    mic_start(&processAudioData);
    desired_position.x = robot.position.x;
    desired_position.y = robot.position.y;
    desired_goal_orientation = robot.orientation;
    chThdSleepMilliseconds(500);
    while(1){
    		if(robot.position.x != desired_position.x || robot.position.y != desired_position.y)
    			move_robot(desired_position.x, desired_position.y,desired_goal_orientation,false, true);
    		else{
    		    desired_position.x = robot.max.x/2;
    		    desired_position.y = robot.max.y/2;
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
	chThdCreateStatic(waGoingtoGoals, sizeof(waGoingtoGoals), NORMALPRIO, GoingtoGoals, NULL);
}




