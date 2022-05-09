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

#define MIN_VALUE_THRESHOLD 10000
#define MIN_FREQ 10 //we don’t analyze before this index to not use resources for nothing
#define FREQ_FORWARD 16 //250Hz
#define FREQ_LEFT 19 //296Hz
#define FREQ_RIGHT 23 //359HZ
#define FREQ_BACKWARD 26 //406Hz
//#define MAX_FREQ 30 //we don’t analyze after this index to not use resources for nothing

#define FREQ1 15
#define FREQ2 20
#define FREQ3 25
#define FREQ4 40
#define FREQ5 45

static float highest_amp_left = MIN_VALUE_THRESHOLD;
static float highest_amp_right = MIN_VALUE_THRESHOLD;
static float highest_amp_front = MIN_VALUE_THRESHOLD;
static float highest_amp_back = MIN_VALUE_THRESHOLD;
static float highest_amplitude = MIN_VALUE_THRESHOLD;
static int16_t highest_index = 0;

/*
*	Callback called when the demodulation of the four microphones is done.
*	We get 160 samples per mic every 10ms (16kHz)
*
*	params :
*	int16_t *data			Buffer containing 4 times 160 samples. the samples are sorted by micro
*							so we have [micRight1, micLeft1, micBack1, micFront1, micRight2, etc...]
*	uint16_t num_samples	Tells how many data we get in total (should always be 640)
*/
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
			//LED_sound_control(num_samples);
			chBSemSignal(&sendToComputer_sem);
			k=0;
		}
		j=0;
		++k;
		motor_sound_command(micRight_output,micLeft_output,micBack_output,micFront_output);
	}
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

void motor_sound_command(float* data_right,float* data_left,float* data_back,float* data_front)
{
	static int16_t old_highest_index;
	//highest amplitudes of the 4 microphones
	highest_amp_right = MIN_VALUE_THRESHOLD;
	highest_amp_left = MIN_VALUE_THRESHOLD;
	highest_amp_front = MIN_VALUE_THRESHOLD;
	highest_amp_back = MIN_VALUE_THRESHOLD;
	int16_t highest_index_right = -1;
	int16_t highest_index_left = -1;
	int16_t highest_index_back = -1;
	int16_t highest_index_front = -1;
	/*uint8_t right = 0;
	uint8_t left = 1;
	uint8_t back = 2;
	uint8_t front = 3;*/

	for(uint16_t i = MIN_FREQ ; i <= FREQ5 ; i++)
	{
		if(data_right[i] > highest_amp_right)
		{
			highest_index_right = i;
			highest_amp_right = data_right[i];
		}
		if(data_left[i] > highest_amp_left)
		{
			highest_index_left = i;
			highest_amp_left = data_left[i];
		}
		if(data_back[i] > highest_amp_back)
		{
			highest_index_back = i;
			highest_amp_back = data_back[i];
		}
		if(data_front[i] > highest_amp_front)
		{
			highest_index_front = i;
			highest_amp_front = data_front[i];
		}
	}
	if((highest_amp_front>highest_amp_back) & (highest_amp_front>highest_amp_left) & (highest_amp_front>highest_amp_right))
	{
		highest_amplitude = highest_amp_front;
	}
	else if ((highest_amp_back>highest_amp_right) & (highest_amp_front>highest_amp_left))
	{
		highest_amplitude = highest_amp_back;
	}
	else if(highest_amp_left>highest_amp_right)
	{
		highest_amplitude = highest_amp_left;
	}
	else
	{
		highest_amplitude = highest_amp_right;
	}
	/*uint8_t highest_side = 0;
	float highest = MIN_VALUE_THRESHOLD;
	if((highest_right>=highest_left) & (highest_right>=highest_back) & (highest_right>=highest_front))
	{
		highest_side = right;
		highest = highest_right;
	}
	else if ((highest_left>=highest_back) & (highest_left>=highest_front))
	{
		highest_side = left;
		highest = highest_left;
	}
	else if(highest_back>=highest_front)
	{
		highest_side = back;
		highest = highest_back;
	}
	else
	{
		highest_side = front;
		highest = highest_front;
	}
	*/
	highest_index = (int16_t)(highest_index_right+highest_index_left+highest_index_back+highest_index_front)/4;
	highest_index = (int16_t) ((highest_index+old_highest_index)/2);
	old_highest_index = highest_index;
	/*if(highest_index<=MAX_FREQ)
	{
		left_motor_set_speed(0);
		right_motor_set_speed(0);
	}
	else if((highest_index>=FREQ2) & (highest_index<=MAX_FREQ))
	{
		left_motor_set_speed(-800);
		right_motor_set_speed(800);
	}

	else if((highest_index>=MAX_FREQ) & (highest > MIN_AMPLITUDE)) //& (highest_index<=FREQ5))
	{
		if(highest_side == front)
		{
			left_motor_set_speed(800);
			right_motor_set_speed(800);
		}
		if(highest_side == back)
		{
			left_motor_set_speed(-800);
			right_motor_set_speed(-800);
		}
		if(highest_side == right)
		{
			left_motor_set_speed(800);
			right_motor_set_speed(-800);
		}
		if(highest_side == left)
		{
			left_motor_set_speed(-800);
			right_motor_set_speed(800);
		}

	}
	else if(highest_index>=FREQ5)
	{
		left_motor_set_speed(0);
		right_motor_set_speed(0);
	}*/

}

float get_right_amplitude(void)
{
	return highest_amp_right;
}

float get_left_amplitude(void)
{
	return highest_amp_left;
}

uint16_t get_highest_index(void)
{
	return highest_index;
}

float get_front_amplitude(void)
{
	return highest_amp_front;
}

float get_back_amplitude(void)
{
	return highest_amp_back;
}

float get_highest_amplitude(void)
{
	return highest_amplitude;
}
