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

static float goal_amplitude = 110000;
static uint32_t correction = 10000;
static uint16_t turn_around = 800;

//simple PI regulator implementation
int16_t pi_regulator(float amplitude, float goal){

	float speed = 0;

	static int32_t sum_error = 0;
	float error = 0;

	error = goal - amplitude;

	//disables the PI regulator if the error is too small
	//this avoids to always move as we cannot exactly be where we want and
	//the mics are a bit noisy
	if((fabs(error) < ERROR_THRESHOLD) || (get_back_amplitude()>get_front_amplitude())){
		return 0;
	}
	uint32_t error1 = (uint32_t) error;
	error1 /= correction;
	if(get_highest_index()>=MAX_FREQ)
	{
		sum_error += error1;
	}
	else
	{
		return 0;
	}

	//we set a maximum and a minimum for the sum to avoid an uncontrolled growth

	if(sum_error > MAX_SUM_ERROR){
		sum_error = MAX_SUM_ERROR;
	}else if(sum_error < -MAX_SUM_ERROR){
		sum_error = -MAX_SUM_ERROR;
	}

	speed = KP * error1 + KI * sum_error;

    return (int16_t)speed;
}

static THD_WORKING_AREA(waPiRegulator, 256);
static THD_FUNCTION(PiRegulator, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    int16_t speed = 0;
    int16_t speed_correction = 0;

    while(1){
        time = chVTGetSystemTime();

        //computes the speed to give to the motors
        //distance_cm is modified by the image processing thread
        speed = pi_regulator(get_highest_amplitude(), goal_amplitude);
        //computes a correction factor to let the robot rotate to be in front of the line
        //speed_correction = (get_right_amplitude() - get_left_amplitude());
        //speed_correction /= correction;
        if(get_back_amplitude()>get_front_amplitude())
        {
        	speed_correction = turn_around;
        }
        else if((abs(speed_correction) < ROTATION_THRESHOLD) || get_highest_index()<MAX_FREQ)
        {
        	speed_correction = 0;
        }
        else
        {
        	speed_correction = (get_right_amplitude() - get_left_amplitude());
        }
        //if the line is nearly in front of the camera, don't rotate
        /*
        if(get_back_amplitude()>=get_front_amplitude())
        {
        	right_motor_set_pos(13);
        	left_motor_set_pos(-13);
        }
        */
		//applies the speed from the PI regulator and the correction for the rotation

		right_motor_set_speed(speed - speed_correction*ROTATION_COEFF);
		left_motor_set_speed(speed + speed_correction*ROTATION_COEFF);

		//right_motor_set_speed(speed-ROTATION_COEFF * speed_correction);
		//left_motor_set_speed(speed+ROTATION_COEFF * speed_correction);

        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

void pi_regulator_start(void){
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, NULL);
}



