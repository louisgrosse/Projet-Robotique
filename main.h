#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "camera/dcmi_camera.h"
#include "msgbus/messagebus.h"
#include "parameter/parameter.h"

#define ROTATION_THRESHOLD		0.15f
#define ROTATION_COEFF			300     // convert rotation speed to be between 100 and 300
#define PXTOCM					1570.0f //experimental value
#define ERROR_THRESHOLD			15000
#define KP						700.0f
#define KI 						3.5f	//must not be zero
#define MAX_SUM_ERROR 			(MOTOR_SPEED_LIMIT/KI)
#define MOV_FREQ 				75
#define goal_amplitude 			140000
#define correction 				10000
#define speed_conversion 		8  		//convert the speed given by the pi_regulator to be between 400 and 800
#define prox_distance_front		125
#define prox_distance_side		25
#define prox_distance			60
#define prox_speed				300
#define PHASE_THRESHOLD			4.5f	//Phase threshold
#define MIN_VALUE_THRESHOLD 	10000   //Amplitude threshold

/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

#ifdef __cplusplus
}
#endif

#endif
