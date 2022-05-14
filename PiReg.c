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

static bool MOVE = FALSE;

//PI regulator implementation
int16_t Pi_Reg(float amplitude, float goal)
{
	//this function takes care only of the distance between the target and the robot (not the angle)
	float speed = 0;

	static int32_t sum_error = 0;
	float error = 0;

	error = goal - amplitude;

	if(fabsf(error) < ERROR_THRESHOLD)
	{
		//set_body_led(1);
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
		//set_body_led(1);
		return 0;
	}

	if(sum_error > MAX_SUM_ERROR)
	{
		sum_error = MAX_SUM_ERROR;
	}
	else if(sum_error < -MAX_SUM_ERROR)
	{
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
    float speed_correction = 0;
    uint16_t FREQ = 0;



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
        bool no_obstacle_detected = ((get_prox_mean_right() < prox_distance) & (get_prox_mean_left() < prox_distance));
        bool obstacle_in_front = ((get_prox_front_right() > prox_distance) || (get_prox_front_left() > prox_distance));
        bool obstacle_left = ((get_prox_left() > prox_distance) & (speed_correction > ROTATION_THRESHOLD) & MOVE);
        bool obstacle_right = ((get_prox_right() > prox_distance) & (speed_correction < -ROTATION_THRESHOLD) & MOVE);
        bool obstacle_on_side = (obstacle_left || obstacle_right);

        if(!MOVE || (get_highest_amplitude() < MIN_VALUE_THRESHOLD))
        {
        	speed=0;
        	speed_correction=0;
        }
        else if(no_obstacle_detected & !obstacle_on_side)
        {
        	speed = Pi_Reg(get_highest_amplitude(), goal_amplitude);

        	if(fabsf(speed_correction) < ROTATION_THRESHOLD)
			{
				speed_correction = 0;
			}
			else if(speed_correction >= 0)
			{
				speed_correction = ROTATION_COEFF;
				//speed = 0;  //Mode 2
			}
			else if(speed_correction < 0)
			{
				speed_correction = -ROTATION_COEFF;
				//speed = 0;   //Mode 2
			}

			set_body_led(0);//test
        }
        else if (get_prox_front_right() < get_prox_right() || get_prox_front_left() < get_prox_left())
        {
        	speed = prox_speed;
        	if(get_prox_front_right > prox_distance)
        	{
        		speed_correction = prox_speed;
        	}
        	else if(get_prox_front_left > prox_distance)
        	{
        		speed_correction = prox_speed;
        	}
        	else
        	{
        		speed_correction = 0;
        	}
        }
        else if (obstacle_on_side || obstacle_in_front)
        {
        	speed = 400;
        	if(get_prox_right() >= get_prox_left())
        	{
        		speed_correction = -get_prox_front_right()*2-(get_prox_side_right()+get_prox_right())/2-get_prox_back_right();
        	}
        	else
        	{
        		speed_correction = get_prox_front_left()*2 + (get_prox_side_left()+get_prox_left())/2 + get_prox_back_left();
        	}

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
