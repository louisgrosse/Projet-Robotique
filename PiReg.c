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

	float speed = 0;

	static int32_t sum_error = 0;
	float error = 0;

	error = goal - amplitude;

	if((fabs(error) < ERROR_THRESHOLD) || (get_back_amplitude()>get_front_amplitude())){
		return 0;
	}
	int32_t error1 = (int32_t) error;
	error1 /= correction;
	if(get_highest_index()>MAX_FREQ)
	{
		sum_error += error1;
	}
	else
	{
		return 0;
	}

	if(sum_error > MAX_SUM_ERROR){
		sum_error = MAX_SUM_ERROR;
	}else if(sum_error < -MAX_SUM_ERROR){
		sum_error = -MAX_SUM_ERROR;
	}

	speed = KP * error1 + KI * sum_error;
	speed = (int16_t) speed;
	speed /= 5;
    return speed;
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

        speed = pi_regulator(get_highest_amplitude(), goal_amplitude);

        /*
        else if(get_back_amplitude()>get_front_amplitude())
        {
        	speed_correction = turn_around;
        	if(get_right_amplitude() < get_left_amplitude())
        	{
        		speed_correction = -speed_correction;
        	}
        }

        if((abs(speed_correction) < ROTATION_THRESHOLD))
        {
        	//speed_correction = (get_right_amplitude() - get_left_amplitude());
        	//speed_correction /= 10000;
        	speed_correction = 0;

        }
        */

        speed_correction = get_dephasage();

        if(get_highest_index()<MAX_FREQ || abs(speed_correction) < ROTATION_THRESHOLD)
		{
			speed_correction = 0;
		}
        else
        {
        	speed_correction *= ROTATION_COEFF;
        	speed_correction = (int16_t) speed_correction;
        }

		right_motor_set_speed(speed + speed_correction);
		left_motor_set_speed(speed - speed_correction);

        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

void pi_regulator_start(void)
{
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, NULL);
}



