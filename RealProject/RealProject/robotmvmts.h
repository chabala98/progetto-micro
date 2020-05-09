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
	float orientation;
} Goal;

//input:	 -
//output: -
//purpose: initialises robotmvmts thread
void robotmvmts_start(void);



#endif /* ROBOTMVMTS_H_ */
