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
static unsigned int prox_side_right = 0;
static unsigned int prox_side_left = 0;

static THD_WORKING_AREA(waAvoidObstacle, 1024);
static THD_FUNCTION(AvoidObstacle, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    messagebus_topic_t *prox_topic = messagebus_find_topic_blocking(&bus, "/proximity");
    proximity_msg_t prox_values;
   	//int16_t leftSpeed = 0, rightSpeed = 0;

    //set_body_led(1);

    while(1){
    	messagebus_topic_wait(prox_topic, &prox_values, sizeof(prox_values));
    	unsigned int front_right= get_prox(0);
    	unsigned int side_right= get_prox(1);
    	unsigned int front_left= get_prox(6);
    	unsigned int side_left= get_prox(7);
    	//unsigned int left = get_prox(5);
    	//unsigned int right = get_prox(2);

    	prox_front_right = front_right;
    	prox_front_left = front_left;
    	prox_side_right = side_right;
    	prox_side_left = side_left;
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

unsigned int get_prox_side_right(void)
{
	return prox_side_right;
}

unsigned int get_prox_side_left(void)
{
	return prox_side_left;
}

void avoid_obstacle_start(void)
{
	chThdCreateStatic(waAvoidObstacle, sizeof(waAvoidObstacle), NORMALPRIO, AvoidObstacle, NULL);
}
