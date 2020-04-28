#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "camera/dcmi_camera.h"
#include "msgbus/messagebus.h"
#include "parameter/parameter.h"


//constants for the differents parts of the project
#define IMAGE_BUFFER_SIZE		640
#define WIDTH_SLOPE				5
#define MIN_LINE_WIDTH			40
#define ROTATION_THRESHOLD		10
#define ROTATION_COEFF			2 
#define PXTOCM					1570.0f //experimental value
#define CMTOMM					10 //experimental value
#define GOAL_DISTANCE 			10.0f
#define MAX_DISTANCE 			25.0f
#define ERROR_THRESHOLD			0.1f	//[cm] because of the noise of the camera
#define KP						800.0f
#define KI 						3.5f	//must not be zero
#define MAX_SUM_ERROR 			(MOTOR_SPEED_LIMIT/KI)
#define DIAMETER_ROBOT 			70 //mm
#define STEP_POSITION 25
#define DIST_CAMERA_MEASURE 160
#define SECURE_DIST 100
#define SIN20CTE 0.34202014
#define COS20CTE 0.93969262
#define TAN_CAMERA_APERTURE 0.42922
#define	NOT_RIGHT_COLOR	60
#define	YIELD_DISTINGUISH_COLOR	200
#define	NB_GOALS		3
#define	RED		0
#define	BLUE 	1
#define	GREEN	2
#define	WHITE	3
#define	COLOR_NOT_ATTRIBUTED		4
#define	OFF						5




/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

void SendUint8ToComputer(uint8_t* data, uint16_t size);

#ifdef __cplusplus
}
#endif

#endif
