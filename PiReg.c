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
#include <selector.h>

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
		set_body_led(1);
		return 0;
	}
	else
	{
		set_body_led(0);
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

    float speed_correction = 0; //factor that will determine the rotation of the robot

    uint16_t FREQ = 0;

    while(1)
    {
        time = chVTGetSystemTime();

        speed_correction = get_dephasage();

        FREQ = get_highest_index();

        MOVE = ((FREQ > MOV_FREQ) & (get_selector()>7) & (get_highest_amplitude() > MIN_VALUE_THRESHOLD));

        //mainly for readability purposes

		bool obstacle_in_front = ((get_prox_front_right() > prox_distance) || (get_prox_front_left() > prox_distance));
        bool obstacle_left = (get_prox_left() > prox_distance);
        bool obstacle_right = (get_prox_right() > prox_distance);
        bool obstacle_on_side = (obstacle_left || obstacle_right);

        //corrects the trajectory of the robot if it got too close to an obstacle
        bool prox_correction = (get_prox_front_right() < get_prox_right() || get_prox_front_left() < get_prox_left());


        bool obstacle_in_back = ((get_prox_back_right() > prox_distance) || (get_prox_back_left() > prox_distance));;

        bool no_obstacle_detected = ((get_prox_mean_right() < prox_distance) & (get_prox_mean_left() < prox_distance) & !obstacle_in_front);

        if(!MOVE)
        {
        	speed=0;
        	speed_correction=0;
        }
        else if(no_obstacle_detected & !obstacle_on_side)
        {
        	speed = Pi_Reg(get_highest_amplitude(), goal_amplitude);

        	if(fabsf(speed_correction) < 2*ROTATION_THRESHOLD)
			{
				speed_correction = 0;
			}
			else if(speed_correction >= 0)
			{
				speed_correction = speed/3; //if the robot is far it'll rotate faster
				//speed = 0;  //Mode 2
			}
			else if(speed_correction < 0)
			{
				speed_correction = -speed/3;
				//speed = 0;   //Mode 2
			}
        }
        else if (prox_correction)
        {
        	//if the robot got too close to a wall it will turn away from it
        	if(get_prox_front_right() > 20)
        	{
        		//the robot got too close to an obstacle on its right
        		speed = 200;
        		speed_correction = -2*prox_speed-100;
        	}
        	else if(get_prox_front_left() > 20)
        	{
        		//the robot got too close to an obstacle on its right
        		speed = 200;
        		speed_correction = 2*prox_speed+100;
        	}
        	else
        	{
        		//the wall is far away so the robot can just go straight
        		speed = 2*prox_speed;
        		speed_correction = 0;
        	}
        }
        else if (obstacle_on_side || obstacle_in_front || obstacle_in_back)
        {
        	//an obstacle is detected and we wrote an equation that will describe the rotation depending on each captor and their weight
        	speed = ROTATION_COEFF;
        	if(get_prox_front_right() >= get_prox_front_left())
        	{
        		speed_correction = -get_prox_front_right()*2-get_prox_mean_right()-get_prox_back_right()/2;
        	}
        	else
        	{
        		speed_correction = get_prox_front_left()*2 + get_prox_mean_left() + get_prox_back_left()/2;
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
