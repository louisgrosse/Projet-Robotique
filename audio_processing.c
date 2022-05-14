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

#define MIN_FREQ 		80
#define MAX_FREQ 		90
#define MIN_FREQ_LOW	20
#define MAX_FREQ_LOW	30

//Highest amplitudes of the mics
static float highest_amp_left = MIN_VALUE_THRESHOLD;
static float highest_amp_right = MIN_VALUE_THRESHOLD;
static float highest_amp_front = MIN_VALUE_THRESHOLD;
static float highest_amp_back = MIN_VALUE_THRESHOLD;
static float highest_amplitude = MIN_VALUE_THRESHOLD;

//Index of the highest amplitude
static int16_t highest_index_right = -1;
static int16_t highest_index_left = -1;
static int16_t highest_index_back = -1;
static int16_t highest_index_front = -1;

static int16_t highest_index = 0;
static float dephasage = 0;

/*
*	Callback called when the demodulation of the four microphones is done.
*	We get 160 samples per mic every 10ms (16kHz)
*
*	params :
*	int16_t *data			Buffer containing 4 times 160 samples. the samples are sorted by micro
*							so we have [micRight1, micLeft1, micBack1, micFront1, micRight2, etc...]
*	uint16_t num_samples	Tells how many data we get in total (should always be 640)
*/

void phase_calculation(void)
{

		//phase of the right and left microphones
		static double last_dephasage = 0;
		double phase_right = 0;
		double phase_left = 0;

		//search for the highest peak

		//max_norm_left = micLeft_output[MaxNormIndex_real_right];

		int16_t MaxNormIndex_cmplx_right = -1;

		int16_t highest_index_right_test = highest_index_right;
		int16_t highest_index_left_test = highest_index_left;

		if(highest_index_right_test!=0)
		{
			highest_index_right_test*=2;
		}

		if(highest_index_right_test!=0)
		{
			highest_index_left_test*=2;
		}

		MaxNormIndex_cmplx_right = highest_index_right_test + 1;

		if(highest_amp_right > MIN_VALUE_THRESHOLD)
		{
			phase_right = atan2(micRight_cmplx_input[MaxNormIndex_cmplx_right],micRight_cmplx_input[highest_index_right_test]);
			phase_left = atan2(micLeft_cmplx_input[MaxNormIndex_cmplx_right],micLeft_cmplx_input[highest_index_right_test]);
			dephasage = phase_right-phase_left;
		}
		if (dephasage < -PHASE_THRESHOLD || dephasage > PHASE_THRESHOLD)
		{
			dephasage = fabs(phase_right) - fabs(phase_left);
			if(last_dephasage > 0)
			{
			  dephasage = fabs(dephasage);
			}
			else
			{
			  dephasage= -fabs(dephasage);
			}
		}

}

void processAudioData(int16_t *data, uint16_t num_samples)
{

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
		phase_calculation();
		//chThdCreateStatic(waFollowSound, sizeof(waFollowSound), NORMALPRIO, FollowSound, NULL);
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
	//static int16_t old_highest_index;
	highest_amp_right = MIN_VALUE_THRESHOLD;
	highest_amp_left = MIN_VALUE_THRESHOLD;
	highest_amp_front = MIN_VALUE_THRESHOLD;
	highest_amp_back = MIN_VALUE_THRESHOLD;

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

	for(uint16_t i = MIN_FREQ_LOW ; i <= MAX_FREQ_LOW ; i++)
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
	if((highest_amp_front > highest_amp_back) & (highest_amp_front > highest_amp_left) & (highest_amp_front > highest_amp_right))
	{
		highest_amplitude = highest_amp_front;
		highest_index = highest_index_front;
	}
	else if ((highest_amp_back > highest_amp_right) & (highest_amp_front > highest_amp_left))
	{
		highest_amplitude = highest_amp_back;
		highest_index = highest_index_back;
	}
	else if(highest_amp_left > highest_amp_right)
	{
		highest_amplitude = highest_amp_left;
		highest_index = highest_index_left;
	}
	else
	{
		highest_amplitude = highest_amp_right;
		highest_index = highest_index_right;
	}
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

/*
void start_phase(void)
{
	chThdCreateStatic(waFollowSound, sizeof(waFollowSound), NORMALPRIO, FollowSound, NULL);
}
*/


