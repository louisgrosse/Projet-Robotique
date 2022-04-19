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
#define MAX_FREQ 30 //we don’t analyze after this index to not use resources for nothing
static float MAX_AMP=15000;

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
	}
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
	motor_sound_command(micRight_cmplx_input,num_samples);
}


void wait_send_to_computer(void){
	chBSemWait(&sendToComputer_sem);
}

float* get_audio_buffer_ptr(BUFFER_NAME_t name){
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

void motor_sound_command(float* data,uint16_t num_samples)
{

	float highest = MIN_VALUE_THRESHOLD;
	int16_t highest_index = -1;
	for(uint16_t i = 0 ; i <= num_samples ; i++){
		if(data[i] > highest){
			highest = data[i];
			highest_index = i;
		}
	}
	if(highest>600)
	{
		left_motor_set_speed(0);
		right_motor_set_speed(0);
	}
	else if((highest>400) & (highest<=600))
	{
		left_motor_set_speed(-1000);
		right_motor_set_speed(1000);
	}
	else if((highest>200) & (highest<400))
	{
		left_motor_set_speed(1000);
		right_motor_set_speed(-1000);
	}
	else if(highest<200)
	{
		left_motor_set_speed(0);
		right_motor_set_speed(0);
	}

	/*
	if(highest>MAX_AMP)
	{
		left_motor_set_speed(1000);
		right_motor_set_speed(-1000);
	}
	else
	{
		left_motor_set_speed(0);
		right_motor_set_speed(0);
	}
	*/

}
/*
void LED_sound_control(uint16_t num_samples)
{
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;

}
*/
