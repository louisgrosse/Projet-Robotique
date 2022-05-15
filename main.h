#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "camera/dcmi_camera.h"
#include "msgbus/messagebus.h"
#include "parameter/parameter.h"

//Defines the speed and rotation speed
#define ROTATION_THRESHOLD		0.15f
#define ROTATION_COEFF			300     // converts rotation speed to be between 100 and 300

#define ERROR_THRESHOLD			15000

//PID options (defines the reactivity of the application)
#define KP						700.0f
#define KI 						3.5f
#define MAX_SUM_ERROR 			(MOTOR_SPEED_LIMIT/KI)
#define goal_amplitude 			140000
#define correction 				10000 //changes the order of magnitude of the microphones

//Thresholds
#define MOV_FREQ 				75		//Frequency threshold
#define MIN_VALUE_THRESHOLD 	10000   //Amplitude threshold
#define PHASE_THRESHOLD			4.5f	//Phase threshold


#define speed_conversion 		8  		//convert the speed given by the pi_regulator to be between 200 and 600

//Defines the reactivity and speed of the proximity sensors
#define prox_distance_front		125
#define prox_distance_side		25
#define prox_distance			60
#define prox_speed				300

/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

#ifdef __cplusplus
}
#endif

#endif
