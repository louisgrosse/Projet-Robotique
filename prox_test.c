#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "chprintf.h"
#include "hal.h"
#include "shell.h"

#include "aseba_vm/aseba_node.h"
#include "aseba_vm/skel_user.h"
#include "aseba_vm/aseba_can_interface.h"
#include "aseba_vm/aseba_bridge.h"
#include "audio/audio_thread.h"
#include "audio/play_melody.h"
#include "audio/play_sound_file.h"
#include "audio/microphone.h"
#include "camera/po8030.h"
#include "epuck1x/Asercom.h"
#include "epuck1x/Asercom2.h"
#include "epuck1x/a_d/advance_ad_scan/e_acc.h"
#include "sensors/battery_level.h"
#include "sensors/imu.h"
#include "sensors/mpu9250.h"
#include "sensors/proximity.h"
#include "sensors/VL53L0X/VL53L0X.h"
#include "cmd.h"
#include "config_flash_storage.h"
#include "exti.h"
#include "i2c_bus.h"
#include "ir_remote.h"
#include "leds.h"
#include "memory_protection.h"
#include "motors.h"
#include "sdio.h"
#include "selector.h"
#include "spi_comm.h"
#include "usbcfg.h"
#include "communication.h"
#include "uc_usage.h"
#include <msgbus/messagebus.h>
#include <prox_test.h>

//uncomment to send the FFTs results from the real microphones
//#define SEND_FROM_MIC

//uncomment to use double buffering to send the FFT to the computer
//#define DOUBLE_BUFFERING

//#define SHELL_WA_SIZE   THD_WORKING_AREA_SIZE(2048)


uint8_t rgb_state = 0, rgb_counter = 0;
uint16_t melody_state = 0, melody_counter = 0;

//messagebus_t bus;
//MUTEX_DECL(bus_lock);
//CONDVAR_DECL(bus_condvar);

static THD_WORKING_AREA(waProx, 2048);
static THD_FUNCTION(Prox, arg)
{

    chRegSetThreadName(__FUNCTION__);
    (void) arg;

    //uint8_t stop_loop = 0;
    systime_t time;

    messagebus_topic_t *prox_topic = messagebus_find_topic_blocking(&bus, "/proximity");
    proximity_msg_t prox_values;
    int16_t leftSpeed = 0, rightSpeed = 0;

    while(1) {
    	time = chVTGetSystemTime();

				messagebus_topic_wait(prox_topic, &prox_values, sizeof(prox_values));
				leftSpeed = MOTOR_SPEED_LIMIT - prox_values.delta[0]*2 - prox_values.delta[1]*1;
				rightSpeed = MOTOR_SPEED_LIMIT - prox_values.delta[7]*2 - prox_values.delta[6]*1;
				right_motor_set_speed(rightSpeed);
				left_motor_set_speed(leftSpeed);

	            switch(rgb_state) {
					case 0: // Red.
						set_rgb_led(0, 10, 0, 0);
						set_rgb_led(1, 10, 0, 0);
						set_rgb_led(2, 10, 0, 0);
						set_rgb_led(3, 10, 0, 0);
						break;
					case 1: // Green.
						set_rgb_led(0, 0, 10, 0);
						set_rgb_led(1, 0, 10, 0);
						set_rgb_led(2, 0, 10, 0);
						set_rgb_led(3, 0, 10, 0);
						break;
					case 2: // Blue.
						set_rgb_led(0, 0, 0, 10);
						set_rgb_led(1, 0, 0, 10);
						set_rgb_led(2, 0, 0, 10);
						set_rgb_led(3, 0, 0, 10);
						break;
	            }
				rgb_counter++;
				if(rgb_counter == 100) {
					rgb_counter = 0;
					rgb_state = (rgb_state+1)%3;
					set_body_led(2);
					set_front_led(2);
				}

				melody_counter++;
				if(melody_counter == 2000) {
					melody_counter = 0;
					melody_state = (melody_state+1)%NB_SONGS;
					playMelody(melody_state, ML_SIMPLE_PLAY, NULL);
				}

				chThdSleepUntilWindowed(time, time + MS2ST(10)); // Refresh @ 100 Hz.
    }
}


void prox_start(void){
	chThdCreateStatic(waProx, sizeof(waProx), NORMALPRIO, Prox, NULL);
}

