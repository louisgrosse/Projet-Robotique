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
#include <Avoid_Obstacle.h>
#include <leds.h>

volatile static bool MOVE;

//PI regulator implementation
int16_t Pi_Reg(float amplitude, float goal)
{
	//this function takes care only of the distance between the target and the robot (not the angle)
	float speed = 0;

	static int32_t sum_error = 0;
	float error = 0;

	error = goal - amplitude;

	if((fabsf(error) < ERROR_THRESHOLD) || (get_back_amplitude() > get_front_amplitude()))
	{
		return 0;
	}

	int32_t error1 = (int32_t) error;
	error1 /= correction;

	if(MOVE)
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
	speed /= speed_conversion;
    return speed;
}

static THD_WORKING_AREA(waPiRegulator, 256);
static THD_FUNCTION(PiRegulator, arg)
{
    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    int16_t speed = 0;
    volatile float speed_correction = 0;
    volatile uint16_t FREQ = 0;

    while(1)
    {
        time = chVTGetSystemTime();

        speed_correction = get_dephasage();

        FREQ = get_highest_index();

        /*
        uint16_t a = FREQ;
        uint16_t b = MOV_FREQ;
        volatile float c = fabsf(speed_correction);
        float d = ROTATION_THRESHOLD;

        volatile unsigned int e = get_prox_front_right();
        volatile unsigned int f = get_prox_front_left();
        */

        //mainly for readability purposes
        MOVE = FREQ > MOV_FREQ;
        volatile bool no_obstacle_detected = ((get_prox_mean_right() < prox_distance) & (get_prox_mean_left() < prox_distance));
        volatile bool obstacle_in_front = ((get_prox_front_right() > prox_distance) || (get_prox_front_left() > prox_distance));
        volatile bool obstacle_left = ((get_prox_left() > prox_distance) & (speed_correction > ROTATION_THRESHOLD) & MOVE);
        volatile bool obstacle_right = ((get_prox_right() > prox_distance) & (speed_correction < -ROTATION_THRESHOLD) & MOVE);
        bool obstacle_on_side = (obstacle_left || obstacle_right);
        //bool obstacle_both_sides = ((get_prox_mean_left() > prox_distance) & (get_prox_mean_right() > prox_distance));

        if(no_obstacle_detected & !obstacle_on_side)
        {
        	speed = Pi_Reg(get_highest_amplitude(), goal_amplitude);

        	if(fabsf(speed_correction) < ROTATION_THRESHOLD)
			{
				speed_correction = 0;
			}
			else if(speed_correction >= 0)
			{
				//speed_correction *= ROTATION_COEFF;
				//speed_correction = (int16_t) speed_correction + ROTATION_COEFF;
				speed_correction = ROTATION_COEFF;
			}
			else if(speed_correction < 0)
			{
				//speed_correction *= ROTATION_COEFF;
				//speed_correction = (int16_t) speed_correction - ROTATION_COEFF;
				speed_correction = -ROTATION_COEFF;
			}

			speed = 0;
			set_body_led(0);//test
        }
        else if (obstacle_on_side)
        {
        	//the obstacle is on the side
        	speed = prox_speed;
        	speed_correction = 0;
        	set_body_led(0);//test
        }
        else if(obstacle_in_front)
        {
			if (get_prox_front_right() > get_prox_front_left())
			{
				speed = 0;
				speed_correction = prox_speed;
			}
			else
			{
				speed = 0;
				speed_correction = -prox_speed;
			}
			set_body_led(0);//test
		}
        else
        {
        	speed = 0;
        	speed_correction = 0;
        }

		if(!MOVE)
		{
			speed = 0;
			speed_correction= 0;
		}

		right_motor_set_speed(speed - speed_correction);
		left_motor_set_speed(speed + speed_correction);

        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

void pi_regulator_start(void)
{
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, NULL);
}



