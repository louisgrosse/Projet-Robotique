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

#define MIN_FREQ 10
#define MAX_FREQ 40

static float highest_amp_left = MIN_VALUE_THRESHOLD;
static float highest_amp_right = MIN_VALUE_THRESHOLD;
static float highest_amp_front = MIN_VALUE_THRESHOLD;
static float highest_amp_back = MIN_VALUE_THRESHOLD;
static float highest_amplitude = MIN_VALUE_THRESHOLD;
static int16_t highest_index = 0;
static float dephasage=0;

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

	 	 chRegSetThreadName(__FUNCTION__);
	    (void)arg;

		float max_norm_right = MIN_VALUE_THRESHOLD;
		float max_norm_left = MIN_VALUE_THRESHOLD;

		int16_t MaxNormIndex_real_right = -1; //index of the reel part of frequency with the highest amplitude of the right sensor
		int16_t MaxNorm_real_left = -1; //index of the reel part of frequency with the highest amplitude of the left sensor
		int16_t MaxNormIndex_cmplx_right = -1; //index of the imaginary part of frequency with the highest amplitude of the right sensor
		int16_t MaxNormIndex_cmplx_left = -1; //index of the imaginary part of frequency with the highest amplitude of the left sensor
		double phase_right;
		double phase_left;

		//search for the highest peak
		for(uint16_t i = 20 ; i <= 50 ; i++)
		{
			if(micRight_output[i] > max_norm_right)
			{
				max_norm_right = micRight_output[i];
				MaxNormIndex_real_right = i;
			}
			/*
			if(micLeft_output[i] > max_norm_left)
			{
				max_norm_left = micLeft_output[i];
				MaxNorm_real_left = i;
			}
		*/
		}

		max_norm_left = micLeft_output[MaxNormIndex_real_right];

		float max_norm_right_check = max_norm_right;
		float max_norm_left_check = max_norm_left;

		if(MaxNormIndex_real_right!=0)
		{
			MaxNormIndex_real_right*=2;
		}

		if(MaxNormIndex_real_right!=0)
		{
			MaxNorm_real_left*=2;
		}

		MaxNormIndex_cmplx_right = MaxNormIndex_real_right + 1;
		MaxNormIndex_cmplx_left = MaxNorm_real_left + 1;

		if((max_norm_right_check>2*MIN_VALUE_THRESHOLD) || (max_norm_left_check>2*MIN_VALUE_THRESHOLD))
		{
			phase_right = atan2(micRight_cmplx_input[MaxNormIndex_cmplx_right],micRight_cmplx_input[MaxNormIndex_real_right]);
			phase_left = atan2(micLeft_cmplx_input[MaxNormIndex_cmplx_left],micLeft_cmplx_input[MaxNorm_real_left]);
			dephasage = phase_right-phase_left;
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
		sound_processing(micRight_output,micLeft_output,micBack_output,micFront_output);
		chThdCreateStatic(waFollowSound, sizeof(waFollowSound), NORMALPRIO, FollowSound, NULL);
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

void sound_processing(float* data_right,float* data_left,float* data_back,float* data_front)
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


	//we search the frequency of the highest amplitude of each mics
	for(uint16_t i = MIN_FREQ ; i <= MAX_FREQ ; i++)
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

	//we search the highest amplitude between the 4 mics
	if((highest_amp_front>highest_amp_back) & (highest_amp_front>highest_amp_left) & (highest_amp_front>highest_amp_right))
	{
		highest_amplitude = highest_amp_front;
		highest_index = highest_index_front;
	}
	else if ((highest_amp_back>highest_amp_right) & (highest_amp_front>highest_amp_left))
	{
		highest_amplitude = highest_amp_back;
		highest_index = highest_index_back;
	}
	else if(highest_amp_left>highest_amp_right)
	{
		highest_amplitude = highest_amp_left;
		highest_index = highest_index_left;
	}
	else
	{
		highest_amplitude = highest_amp_right;
		highest_index = highest_index_right;
	}

	//mean of the 4 mics
	//highest_index = (int16_t)(highest_index_right+highest_index_left+highest_index_back+highest_index_front)/4;
	//mean on 2 samples to smoothen the movements
	highest_index = (int16_t) ((highest_index+old_highest_index)/2);
	old_highest_index = highest_index;

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

float get_dephasage(void)
{
	return dephasage;
}


