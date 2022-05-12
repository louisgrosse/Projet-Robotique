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

static unsigned int prox_right = 0;
static unsigned int prox_left = 0;

static THD_WORKING_AREA(waAvoidObstacle, 1024);
static THD_FUNCTION(AvoidObstacle, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    messagebus_topic_t *prox_topic = messagebus_find_topic_blocking(&bus, "/proximity");
    proximity_msg_t prox_values;
   	//int16_t leftSpeed = 0, rightSpeed = 0;

    while(1){
    	messagebus_topic_wait(prox_topic, &prox_values, sizeof(prox_values));
    	unsigned int front_right= get_prox(0);
    	unsigned int side_right= get_prox(1);
    	unsigned int front_left= get_prox(6);
    	unsigned int side_left= get_prox(7);
    	prox_right= front_right + side_right;
    	prox_left= front_left + side_left;
    		if ((get_prox_right() < 250) & (get_prox_left()<250)){
    			left_motor_set_speed(800);
    			right_motor_set_speed(800);
    		}
    		if ((a_b > 250) || (c_d>250)){
    			if (get_prox_right() > get_prox_left()){
    				left_motor_set_speed(-800);
    				right_motor_set_speed(800);
    			}
    			else{
    				left_motor_set_speed(800);
    				right_motor_set_speed(-800);
    			}
    		}
    }
}

unsigned int get_prox_right(void){
	return prox_right;
}

unsigned int get_prox_left(void){
	return prox_left;
}

void avoid_obstacle_start(void){
	chThdCreateStatic(waAvoidObstacle, sizeof(waAvoidObstacle), NORMALPRIO+1, AvoidObstacle, NULL);
}




