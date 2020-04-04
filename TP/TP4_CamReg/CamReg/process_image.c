#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>

#include <main.h>
#include <camera/po8030.h>

#include <process_image.h>


static float distance_cm = 0;

//semaphore
static BSEMAPHORE_DECL(image_ready_sem, TRUE);

static THD_WORKING_AREA(waCaptureImage, 256);
static THD_FUNCTION(CaptureImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	//Takes pixels 0 to IMAGE_BUFFER_SIZE of the line 10 + 11 (minimum 2 lines because reasons)
	po8030_advanced_config(FORMAT_RGB565, 0, 10, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);
	dcmi_enable_double_buffering();
	dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
	dcmi_prepare();

    while(1){
        //starts a capture
		dcmi_capture_start();
		//systime_t orologio = chVTGetSystemTime();
		//waits for the capture to be done
		wait_image_ready();
		//orologio = chVTGetSystemTime()-orologio;
		//chprintf((BaseSequentialStream *) &SD3, "time : %d \r\n", orologio);
		//signals an image has been captured
		chBSemSignal(&image_ready_sem);
    }
}


static THD_WORKING_AREA(waProcessImage, 1024);
static THD_FUNCTION(ProcessImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	uint8_t *img_buff_ptr;
	uint8_t image[IMAGE_BUFFER_SIZE] = {0};
	bool etat=1;


    while(1){
    	//waits until an image has been captured
        chBSemWait(&image_ready_sem);
		//gets the pointer to the array filled with the last image in RGB565    
		img_buff_ptr = dcmi_get_last_image_ptr();


		/*
		*	To complete
		*/
		for(uint16_t i=0; i < 2*IMAGE_BUFFER_SIZE ; i += 2){
			image [i/2] = (img_buff_ptr[i+1] & 31);
			//image [i/2] |= ((img_buff_ptr[i] >> 5) + ((img_buff_ptr[i+1] & 0b00000111) << 3));
			//img_buff_ptr += sizeof(uint16_t);
		}
		etat =! etat;
		//if(etat)
			//SendUint8ToComputer(image,IMAGE_BUFFER_SIZE);
		distance_cm = 640/get_black_width_pixel(image)/0.42922;
		chprintf((BaseSequentialStream *) &SD3, "length in pixel : %d  length in cm : %lf \r\n", get_black_width_pixel(image),get_distance_cm());


    }
}

uint16_t get_black_width_pixel(uint8_t image[IMAGE_BUFFER_SIZE])
{
	uint16_t black_width_pixel = 0;
	for(uint16_t i=0; i < IMAGE_BUFFER_SIZE-2 ; i ++){
		if(image[i] <= 4 && image[i+1] <= 4 && image[i+2]<=4)
			black_width_pixel++;
		if(image[i] >= 4 && image[i+1] >= 4 && image[i+2] >=4 && black_width_pixel >=5)
			break;
	}
	return black_width_pixel;
}

float get_distance_cm(){
	return distance_cm;
}

void process_image_start(void){
	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);
	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);
}
