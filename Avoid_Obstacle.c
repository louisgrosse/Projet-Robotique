#include "ch.h"
#include "hal.h"
#include <main.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <audio/microphone.h>
#include <audio_processing.h>
#include <communications.h>
#include <fft.h>
#include "sensors/proximity.h"
#include <arm_math.h>

static unsigned int prox_front_right = 0;
static unsigned int prox_front_left = 0;
static unsigned int prox_mean_right = 0;
static unsigned int prox_mean_left = 0;
static unsigned int prox_right = 0;
static unsigned int prox_left = 0;
static unsigned int prox_side_right = 0;
static unsigned int prox_side_left = 0;

static THD_WORKING_AREA(waAvoidObstacle, 1024);
static THD_FUNCTION(AvoidObstacle, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    messagebus_topic_t *prox_topic = messagebus_find_topic_blocking(&bus, "/proximity");
    proximity_msg_t prox_values;

    volatile unsigned int prox_test0 = 0;
    volatile unsigned int prox_test1 = 0;
    volatile unsigned int prox_test2 = 0;
    volatile unsigned int prox_test5 = 0;
    volatile unsigned int prox_test6 = 0;
    volatile unsigned int prox_test7 = 0;
    //set_body_led(1);

    while(1)
    {
    	 time = chVTGetSystemTime();

    	messagebus_topic_wait(prox_topic, &prox_values, sizeof(prox_values));

    	prox_test0 = get_calibrated_prox(0);
    	prox_test1 = get_calibrated_prox(1);
    	prox_test2 = get_calibrated_prox(2);
    	prox_test5 = get_calibrated_prox(5);
    	prox_test6 = get_calibrated_prox(6);
    	prox_test7 = get_calibrated_prox(7);

    	if(prox_test0 > correction)
    	{
    		prox_test0 = prox_front_right;
    	}
    	if(prox_test1 > correction)
		{
			prox_test1 = prox_side_right;
		}
    	if(prox_test2 > correction)
		{
			prox_test2 = prox_right;
		}
    	if(prox_test5 > correction)
		{
			prox_test5 = prox_left;
		}
    	if(prox_test6 > correction)
		{
			prox_test6 = prox_side_left;
		}
    	if(prox_test7 > correction)
		{
			prox_test7 = prox_front_left;
		}

    	prox_front_right = prox_test0;
		prox_front_left = prox_test6;

		prox_mean_right = (prox_test1+prox_test0)/2; //mean between the 2 front right sensors
		prox_mean_left = (prox_test7+get_prox(6))/2;  //mean between the 2 front left sensors

		prox_left = prox_test5;
		prox_right = prox_test2;

    	/*
    	prox_front_right = get_calibrated_prox(0);
    	prox_front_left = get_calibrated_prox(6);

    	prox_mean_right = (get_prox(1)+get_prox(0))/2; //mean between the 2 front right sensors
    	prox_mean_left = (get_prox(7)+get_prox(6))/2;  //mean between the 2 front left sensors

    	prox_left = get_calibrated_prox(5);
    	prox_right = get_calibrated_prox(2);
    	*/

    	chThdSleepUntilWindowed(time, time + MS2ST(10));
    }

}

unsigned int get_prox_front_right(void)
{
	return prox_front_right;
}

unsigned int get_prox_front_left(void)
{
	return prox_front_left;
}

unsigned int get_prox_mean_right(void)
{
	return prox_mean_right;
}

unsigned int get_prox_mean_left(void)
{
	return prox_mean_left;
}

unsigned int get_prox_right(void)
{
	return prox_right;
}

unsigned int get_prox_left(void)
{
	return prox_left;
}

void avoid_obstacle_start(void)
{
	chThdCreateStatic(waAvoidObstacle, sizeof(waAvoidObstacle), NORMALPRIO, AvoidObstacle, NULL);
}
