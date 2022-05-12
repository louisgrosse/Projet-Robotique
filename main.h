#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "camera/dcmi_camera.h"
#include "msgbus/messagebus.h"
#include "parameter/parameter.h"

#define ROTATION_THRESHOLD		0.15
#define ROTATION_COEFF			500
#define PXTOCM					1570.0f //experimental value
//#define GOAL_AMPLITUDE 			200000
#define ERROR_THRESHOLD			15000
#define KP						800.0f
#define KI 						3.5f	//must not be zero
#define MAX_SUM_ERROR 			(MOTOR_SPEED_LIMIT/KI)
#define MAX_FREQ 30
#define goal_amplitude 110000
#define turn_around 800
#define correction 10000

/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

#ifdef __cplusplus
}
#endif

#endif
