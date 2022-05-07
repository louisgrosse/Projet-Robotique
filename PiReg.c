/*
 * PiReg.c
 *
 *  Created on: 5 May 2022
 *      Author: louis
 */
#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>


#include <main.h>
#include <motors.h>
#include <PiReg.h>
#include <audio_processing.h>

//simple PI regulator implementation
int16_t pi_regulator(float amplitude, float goal){

	float error = 0;
	float speed = 0;

	static float sum_error = 0;

	error = amplitude - goal;

	//disables the PI regulator if the error is to small
	//this avoids to always move as we cannot exactly be where we want and
	//the mics are a bit noisy
	if(fabs(error) < ERROR_THRESHOLD){
		return 0;
	}

	sum_error += error;

	//we set a maximum and a minimum for the sum to avoid an uncontrolled growth
	if(sum_error > MAX_SUM_ERROR){
		sum_error = MAX_SUM_ERROR;
	}else if(sum_error < -MAX_SUM_ERROR){
		sum_error = -MAX_SUM_ERROR;
	}

	speed = KP * error + KI * sum_error;

    return (int16_t)speed;
}

static THD_WORKING_AREA(waPiRegulator, 256);
static THD_FUNCTION(PiRegulator, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    int16_t speed = 0;
    float speed_correction = 0;

    while(1){
        time = chVTGetSystemTime();

        //computes the speed to give to the motors
        //distance_cm is modified by the image processing thread
        speed = pi_regulator(get_front_amplitude(), GOAL_AMPLITUDE);
        //computes a correction factor to let the robot rotate to be in front of the line
        speed_correction = (get_right_amplitude() - get_left_amplitude());

        //if the line is nearly in front of the camera, don't rotate
        if(abs(speed_correction) < ROTATION_THRESHOLD){
        	speed_correction = 0;
        }
        if(get_highest_index()<=MAX_FREQ)
        {
        	right_motor_set_speed(0);
			left_motor_set_speed(0);
        }
        else
        {
        	//applies the speed from the PI regulator and the correction for the rotation
			right_motor_set_speed(speed - ROTATION_COEFF * speed_correction);
			left_motor_set_speed(speed + ROTATION_COEFF * speed_correction);
        }

        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

void pi_regulator_start(void){
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, NULL);
}



