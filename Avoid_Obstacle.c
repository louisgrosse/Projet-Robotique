#include "ch.h"
#include "hal.h"
#include <main.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <motors.h>
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

static THD_WORKING_AREA(waAvoidObstacle, 1024);
static THD_FUNCTION(AvoidObstacle, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;
    time = chVTGetSystemTime();

    messagebus_topic_t *prox_topic = messagebus_find_topic_blocking(&bus, "/proximity");
    proximity_msg_t prox_values;
   	//int16_t leftSpeed = 0, rightSpeed = 0;

    //set_body_led(1);

    while(1)
    {
    	messagebus_topic_wait(prox_topic, &prox_values, sizeof(prox_values));

    	prox_front_right = get_prox(0);
    	prox_front_left = get_prox(6);

    	prox_mean_right = (get_prox(1)+get_prox(0))/2; //mean between the 2 front right sensors
    	prox_mean_left = (get_prox(7)+get_prox(6))/2;  //mean between the 2 front left sensors

    	prox_left = get_prox(5);
    	prox_right = get_prox(2);
    }

    chThdSleepUntilWindowed(time, time + MS2ST(10));

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
