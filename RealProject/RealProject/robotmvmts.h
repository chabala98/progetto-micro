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

<<<<<<< HEAD
=======






>>>>>>> 8624660db6ce97125d5ca04f847a51ed0c81d01e
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

<<<<<<< HEAD
//input:	 -
//output: -
//purpose: initialises robotmvmts thread
void robotmvmts_start(void);

bool retrieve_stop_camera(void);
=======

void robotmvmts_start(void);


>>>>>>> 8624660db6ce97125d5ca04f847a51ed0c81d01e

#endif /* ROBOTMVMTS_H_ */
