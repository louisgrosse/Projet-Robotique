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

//semaphore
static BSEMAPHORE_DECL(sendToComputer_sem, TRUE);

//2 times FFT_SIZE because these arrays contain complex numbers (real + imaginary)
static float micLeft_cmplx_input[2 * FFT_SIZE];
static float micRight_cmplx_input[2 * FFT_SIZE];
static float micFront_cmplx_input[2 * FFT_SIZE];
static float micBack_cmplx_input[2 * FFT_SIZE];
//Arrays containing the computed magnitude of the complex numbers
static float micLeft_output[FFT_SIZE];
static float micRight_output[FFT_SIZE];
static float micFront_output[FFT_SIZE];
static float micBack_output[FFT_SIZE];
static float dephasage=0;
//static float angle;

#define MIN_VALUE_THRESHOLD 10000
#define MIN_FREQ 10 //we don’t analyze before this index to not use resources for nothing
#define FREQ_FORWARD 16 //250Hz
#define FREQ_LEFT 19 //296Hz
#define FREQ_RIGHT 23 //359HZ
#define FREQ_BACKWARD 26 //406Hz
#define MAX_FREQ 30 //we don’t analyze after this index to not use resources for nothing

#define FREQ1 15
#define FREQ2 20
#define FREQ3 25
#define FREQ4 40
#define FREQ5 45

/*
*	Callback called when the demodulation of the four microphones is done.
*	We get 160 samples per mic every 10ms (16kHz)
*
*	params :
*	int16_t *data			Buffer containing 4 times 160 samples. the samples are sorted by micro
*							so we have [micRight1, micLeft1, micBack1, micFront1, micRight2, etc...]
*	uint16_t num_samples	Tells how many data we get in total (should always be 640)
*/
static THD_WORKING_AREA(waFollowSound, 1024);
static THD_FUNCTION(FollowSound, arg) {
		float max_norm1 = MIN_VALUE_THRESHOLD;
		float max_norm2 = MIN_VALUE_THRESHOLD;
		int16_t max_norm_index1_1 = -1;
		int16_t max_norm_index2_1 = -1;
		int16_t max_norm_index1_2 = -1;
		int16_t max_norm_index2_2 = -1;
		double argument1;
		double argument2;

		//search for the highest peak
		for(uint16_t i = 20 ; i <= 50 ; i++){
			if(micRight_output[i] > max_norm1){
				max_norm1 = micRight_output[i];
				max_norm_index1_1 = i;
			}
			if(micLeft_output[i] > max_norm2){
				max_norm2 = micLeft_output[i];
				max_norm_index2_1 = i;
			}
		}
		float max_norm1_check = max_norm1;
		float max_norm2_check = max_norm2;
		if(max_norm_index1_1!=0){
			max_norm_index1_1*=2;
		}
		if(max_norm_index1_1!=0){
			max_norm_index2_1*=2;
		}
		max_norm_index1_2 = max_norm_index1_1 + 1;
		max_norm_index2_2 = max_norm_index2_1 + 1;
		if((max_norm1_check>2*MIN_VALUE_THRESHOLD) || (max_norm2_check>2*MIN_VALUE_THRESHOLD)){
			argument1 = atan2(micRight_cmplx_input[max_norm_index1_2],micRight_cmplx_input[max_norm_index1_1]);
			argument2 = atan2(micLeft_cmplx_input[max_norm_index2_2],micLeft_cmplx_input[max_norm_index2_1]);
			dephasage = argument1-argument2;
		}

		if ((get_dephasage()<0.15) & (get_dephasage()>-0.15)){
			left_motor_set_speed(0);
			right_motor_set_speed(0);
		}else if(get_dephasage()>0.15){
			left_motor_set_speed(300);
			right_motor_set_speed(-300);
		}else{
			left_motor_set_speed(-300);
			right_motor_set_speed(300);
		}
}

void processAudioData(int16_t *data, uint16_t num_samples){

	/*
	*
	*	We get 160 samples per mic every 10ms
	*	So we fill the samples buffers to reach
	*	1024 samples, then we compute the FFTs.
	*
	*/
	static uint16_t j=0;
	static uint8_t k =0;
	//float max_norm = MIN_VALUE_THRESHOLD;
	//int16_t max_norm_index = -1;
	for(uint16_t i=0;i<num_samples;i+=4)
	{
		micRight_cmplx_input[j]=(float)data[i];
		micLeft_cmplx_input[j]=(float)data[i+MIC_LEFT];
		micBack_cmplx_input[j]=(float)data[i+MIC_BACK];
		micFront_cmplx_input[j]=(float)data[i+MIC_FRONT];
		++j;
		micRight_cmplx_input[j]=0;
		micLeft_cmplx_input[j]=0;
		micBack_cmplx_input[j]=0;
		micFront_cmplx_input[j]=0;
		++j;
		if(j >= (2 * FFT_SIZE))
		{
			break;
		}
	}
	if(j >= (2 * FFT_SIZE))
	{
		doFFT_optimized(FFT_SIZE, micRight_cmplx_input);
		doFFT_optimized(FFT_SIZE, micLeft_cmplx_input);
		doFFT_optimized(FFT_SIZE, micBack_cmplx_input);
		doFFT_optimized(FFT_SIZE, micFront_cmplx_input);

		arm_cmplx_mag_f32(micRight_cmplx_input, micRight_output, FFT_SIZE);
		arm_cmplx_mag_f32(micLeft_cmplx_input, micLeft_output, FFT_SIZE);
		arm_cmplx_mag_f32(micBack_cmplx_input, micBack_output, FFT_SIZE);
		arm_cmplx_mag_f32(micFront_cmplx_input, micFront_output, FFT_SIZE);

		if(k>8)
		{
			chBSemSignal(&sendToComputer_sem);
			k=0;
		}
		j=0;
		++k;
		chThdCreateStatic(waFollowSound, sizeof(waFollowSound), NORMALPRIO, FollowSound, NULL);
	}
}

float get_dephasage(void){
	return dephasage;
}



void wait_send_to_computer(void)
{
	chBSemWait(&sendToComputer_sem);
}

float* get_audio_buffer_ptr(BUFFER_NAME_t name)
{
	if(name == LEFT_CMPLX_INPUT){
		return micLeft_cmplx_input;
	}
	else if (name == RIGHT_CMPLX_INPUT){
		return micRight_cmplx_input;
	}
	else if (name == FRONT_CMPLX_INPUT){
		return micFront_cmplx_input;
	}
	else if (name == BACK_CMPLX_INPUT){
		return micBack_cmplx_input;
	}
	else if (name == LEFT_OUTPUT){
		return micLeft_output;
	}
	else if (name == RIGHT_OUTPUT){
		return micRight_output;
	}
	else if (name == FRONT_OUTPUT){
		return micFront_output;
	}
	else if (name == BACK_OUTPUT){
		return micBack_output;
	}
	else{
		return NULL;
	}
}
