#include "ch.h"
#include "hal.h"
#include <main.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <motors.h>
#include <audio/microphone.h>
#include <audio_processing.h>
#include <fft.h>
#include <arm_math.h>
#include <leds.h>



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



#define MIN_FREQ		10	//we don't analyze before this index to not use resources for nothing
#define MAX_FREQ		30	//we don't analyze after this index to not use resources for nothing
#define FREQ_GREEN	19 //296Hz
#define FREQ_BLUE	23//359Hz
#define FREQ_RED		26//406Hz
#define MIN_VALUE_THRESHOLD 10000
#define SOUND_THRESHOLD_VALUE  9



#define FREQ_BLUE_L			(FREQ_BLUE-1)
#define FREQ_BLUE_H			(FREQ_BLUE+1)
#define FREQ_RED_L			(FREQ_RED-1)
#define FREQ_RED_H			(FREQ_RED+1)
#define FREQ_GREEN_L			(FREQ_GREEN-1)
#define FREQ_GREEN_H			(FREQ_GREEN+1)

static int8_t goal_destination = COLOR_NOT_ATTRIBUTED;

/***************************INTERNAL FUNCTIONS************************************/
/*
*	Simple function used to detect the highest value in a buffer
*	and to execute a motor command depending on it
*/

//input:	 *data is the Fourier transform of the sound (magnitude in function of the frequency)
//output : -
//purpose: Detect the sounds, and depending on the detected frequency chose the destination goal
void sound_remote(float* data){
	float max_norm = MIN_VALUE_THRESHOLD;
	int16_t max_norm_index = -1;
	static bool green_light = false;
	static bool blue_light = false;
	static bool red_light = false;
	static int8_t nb_entries = 0;

	//search for the highest peak
	for(uint16_t i = MIN_FREQ ; i <= MAX_FREQ ; i++){
		if(data[i] > max_norm){
			max_norm = data[i];
			max_norm_index = i;
		}
	}
	//go to green goal
	if(max_norm_index >= FREQ_GREEN_L && max_norm_index <= FREQ_GREEN_H){
		if(green_light && nb_entries > SOUND_THRESHOLD_VALUE){
			set_rgb_led(LED2, 0, RGB_MAX_INTENSITY, 0);
			set_rgb_led(LED4, 0, RGB_MAX_INTENSITY, 0);
			set_rgb_led(LED6, 0, RGB_MAX_INTENSITY, 0);
			set_rgb_led(LED8, 0, RGB_MAX_INTENSITY, 0);
			goal_destination = GREEN;
		}
		else{
			if(red_light || blue_light || !(green_light)){
				nb_entries = 1;
				green_light = true;
				blue_light = false;
				red_light = false;
			}
			else
				nb_entries++;
		}
	}
	//go to red goal
	else if(max_norm_index >= FREQ_RED_L && max_norm_index <= FREQ_RED_H){
		if(red_light && nb_entries > SOUND_THRESHOLD_VALUE){
			set_rgb_led(LED2, RGB_MAX_INTENSITY, 0, 0);
			set_rgb_led(LED4, RGB_MAX_INTENSITY, 0, 0);
			set_rgb_led(LED6, RGB_MAX_INTENSITY, 0, 0);
			set_rgb_led(LED8, RGB_MAX_INTENSITY, 0, 0);
			goal_destination = RED;
		}
		else{
			if(green_light || blue_light||!(red_light)){
				nb_entries = 1;
				green_light = false;
				blue_light = false;
				red_light = true;
			}
			else
				nb_entries++;
		}
	}
	//go to blue goal
	else if(max_norm_index >= FREQ_BLUE_L && max_norm_index <= FREQ_BLUE_H){
		if(blue_light && nb_entries > SOUND_THRESHOLD_VALUE){
			set_rgb_led(LED2, 0, 0, RGB_MAX_INTENSITY);
			set_rgb_led(LED4, 0, 0, RGB_MAX_INTENSITY);
			set_rgb_led(LED6, 0, 0, RGB_MAX_INTENSITY);
			set_rgb_led(LED8, 0, 0, RGB_MAX_INTENSITY);
			goal_destination = BLUE;
		}
		else{
			if(green_light || red_light ||!(blue_light)){
				nb_entries = 1;
				green_light = false;
				blue_light = true;
				red_light = false;
			}
			else
				nb_entries++;
		}
	}
	else{
		nb_entries = 0;
		set_rgb_led(LED2, 0, 0, 0);
		set_rgb_led(LED4, 0, 0, 0);
		set_rgb_led(LED6, 0, 0, 0);
		set_rgb_led(LED8, 0, 0, 0);
		goal_destination = COLOR_NOT_ATTRIBUTED;
	}
	chprintf((BaseSequentialStream *)&SD3, "colore attribuito : %d\n",goal_destination);
}

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
	static uint16_t nb_samples = 0;
	for(uint16_t i = 0 ; i < num_samples ; i+=4){
		micLeft_cmplx_input[nb_samples] = (float)data[i + MIC_LEFT];
		nb_samples++;
		micLeft_cmplx_input[nb_samples] = 0;
		nb_samples++;
		if(nb_samples >= (2 * FFT_SIZE)){
			break;
		}
	}

	if(nb_samples >= (2 * FFT_SIZE)){
		doFFT_optimized(FFT_SIZE, micLeft_cmplx_input);
		arm_cmplx_mag_f32(micLeft_cmplx_input, micLeft_output, FFT_SIZE);
		nb_samples = 0;
		sound_remote(micLeft_output);
	}
}

//function from TP5
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
/***************************END INTERNAL FUNCTIONS************************************/


/***************************EXTERNAL FUNCTIONS************************************/

//input:	 -
//output : goal_destination
//purpose: return the goal destination

uint8_t get_goal_destination(void){
	return goal_destination;
}
