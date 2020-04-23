/*
 * robotmvmts.h
 *
 *  Created on: 5 Apr 2020
 *      Author: nicolasbaldwin
 */

#ifndef ROBOTMVMTS_H_
#define ROBOTMVMTS_H_

#include <stdint.h>
#include <hal.h>


#define STEP_POSITION 50
#define DIST_CAMERA_MEASURE 160
#define SECURE_DIST 100
#define SIN10CTE 0.17365
#define COS10CTE 0.984808
#define TAN_CAMERA_APERTURE 0.42922




typedef struct {
	uint16_t x;
	uint16_t y;
} Position;

typedef struct {
	Position position;
	Position max;
    float orientation;

} Robot;

typedef struct {
    enum {
        FIRST_GOAL=0,
        SECOND_GOAL=1,
        THIRD_GOAL=2
    } goal_nbrs;
    Position position;
	uint8_t color;
} Goal;

void init_identify_world(void);



#endif /* ROBOTMVMTS_H_ */
