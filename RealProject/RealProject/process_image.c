#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>

#include <main.h>
#include <camera/po8030.h>

#include <process_image.h>
#include <robotmvmts.h>




<<<<<<< HEAD
=======


>>>>>>> 8624660db6ce97125d5ca04f847a51ed0c81d01e
static uint16_t line_position = IMAGE_BUFFER_SIZE/2;	//middle
static uint8_t color_detected;
uint16_t lineWidth = 0;


//semaphore
static BSEMAPHORE_DECL(image_ready_sem, TRUE);

/***************************INTERNAL FUNCTIONS************************************/

//input:	 1)img_buff_ptr points on most recent captured picture  2) image is the most recent picture with only info on one pixel color 3) color we want to analyse
//output: counter value
//purpose: counts the number of pixels containing the selected color in the image
uint16_t determine_color(uint8_t *img_buff_ptr, uint8_t *image,uint8_t color){
	uint16_t counter = 0;
	switch (color)
	{
	    case RED:
	    		//Extracts only the RED pixels
			for(uint16_t i = 0 ; i < (2 * IMAGE_BUFFER_SIZE) ; i+=2){
				image[i/2] = (uint8_t)img_buff_ptr[i]&0xF8;
				if(image[i/2] < NOT_RIGHT_COLOR)
					counter++;
			}
	        break;
	    case BLUE:
			//Extracts only the BLUE pixels
			for(uint16_t i = 0 ; i < (2 * IMAGE_BUFFER_SIZE) ; i+=2){
				image[i/2] = (((uint8_t)img_buff_ptr[i+1]&31)<<3);
				if(image[i/2] < NOT_RIGHT_COLOR)
					counter++;
			}
	        break;
	    case GREEN :
			//Extracts only the GREEN pixels
			for(uint16_t i = 0 ; i < (2 * IMAGE_BUFFER_SIZE) ; i+=2){
				image[i/2] = (((uint8_t)img_buff_ptr[i+1]&224)>>3)  + (((uint8_t)img_buff_ptr[i]&3)<<5);
				if(image[i/2] < NOT_RIGHT_COLOR)
					counter++;
			}
			break;

	    default: break;
	}
	return counter;
}

/*
 *  Returns the line's width extracted from the image buffer given
 *  Returns 0 if line not found
 *
 */


uint16_t determine_color(uint8_t *img_buff_ptr, uint8_t *image,uint8_t color){
	uint16_t counter = 0;
	switch (color)
	{
	 //NOTE :	VALUES AMPLIFIED (0bXXXXX000)
	    case RED:
	    		//Extracts only the RED pixels
			for(uint16_t i = 0 ; i < (2 * IMAGE_BUFFER_SIZE) ; i+=2){
			//extracts first 5bits of the first byte
			//takes nothing from the second byte
				image[i/2] = (uint8_t)img_buff_ptr[i]&0xF8;
				if(image[i/2] < NOT_RIGHT_COLOR)
					counter++;
			}
	        break;
	    case BLUE:
			//Extracts only the BLUE pixels
			for(uint16_t i = 0 ; i < (2 * IMAGE_BUFFER_SIZE) ; i+=2){
			//extracts first 5bits of the first byte
			//takes nothing from the second byte
				image[i/2] = (((uint8_t)img_buff_ptr[i+1]&31)<<3);
				if(image[i/2] < NOT_RIGHT_COLOR)
					counter++;
			}
	        break;
	    case GREEN :
			//Extracts only the GREEN pixels
			for(uint16_t i = 0 ; i < (2 * IMAGE_BUFFER_SIZE) ; i+=2){
			//extracts first 5bits of the first byte
			//takes nothing from the second byte
				image[i/2] = (((uint8_t)img_buff_ptr[i+1]&224)>>3)  + (((uint8_t)img_buff_ptr[i]&3)<<5);
				if(image[i/2] < NOT_RIGHT_COLOR)
					counter++;
			}
			break;

	    default: break;
	}
	return counter;
}
uint16_t extract_line_width(uint8_t *buffer){

	uint16_t i = 0, begin = 0, end = 0, width = 0;
	uint8_t stop = 0, wrong_line = 0, line_not_found = 0;
	uint32_t mean = 0;

	//performs an average
	for(uint16_t i = 0 ; i < IMAGE_BUFFER_SIZE ; i++){
		mean += buffer[i];
	}
	mean /= IMAGE_BUFFER_SIZE;

	do{
		wrong_line = 0;
		//search for a begin
		while(stop == 0 && i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE))
		{ 
			//the slope must at least be WIDTH_SLOPE wide and is compared
		    //to the mean of the image
		    if(buffer[i] > mean && buffer[i+WIDTH_SLOPE] < mean)
		    {
		        begin = i;
		        stop = 1;
		    }
		    i++;
		}
		//if a begin was found, search for an end
		if (i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE) && begin)
		{
		    stop = 0;
		    
		    while(stop == 0 && i < IMAGE_BUFFER_SIZE)
		    {
		        if(buffer[i] > mean && buffer[i-WIDTH_SLOPE] < mean)
		        {
		            end = i;
		            stop = 1;
		        }
		        i++;
		    }
		    //if an end was not found
		    if (i > IMAGE_BUFFER_SIZE || !end)
		    {
		        line_not_found = 1;
		    }
		}
		else//if no begin was found
		{
		    line_not_found = 1;
		}

		//if a line too small has been detected, continues the search
		if(!line_not_found && (end-begin) < MIN_LINE_WIDTH){
			i = end;
			begin = 0;
			end = 0;
			stop = 0;
			wrong_line = 1;
		}
	}while(wrong_line);

	if(line_not_found){
		begin = 0;
		end = 0;
		width = 0;
	}else{
		width = (end - begin);
		line_position = (begin + end)/2; //gives the line position.
	}
		return width;
<<<<<<< HEAD
}


//input:	 1)img_buff_ptr points on most recent captured picture  2) image is the most recent picture with only info on one pixel color
//output : returns the color seen by the camera
//purpose: allows the robot to recognise the color RED,BLUE,GREEN (if its non of the 3, it's white). Therefore it can distinguish the different goals from the wall.
uint8_t determine_goals(uint8_t *img_buff_ptr, uint8_t *image){
	uint16_t red_counter = 0 , green_counter = 0,blue_counter = 0;
	red_counter = determine_color(img_buff_ptr, image,RED);
	green_counter = determine_color(img_buff_ptr, image,GREEN);
	blue_counter = determine_color(img_buff_ptr, image,BLUE);
	if((red_counter < blue_counter) && (red_counter < green_counter)){
		if(((blue_counter-red_counter) > YIELD_DISTINGUISH_COLOR) &&  ((green_counter-red_counter) > YIELD_DISTINGUISH_COLOR)){
			return RED;
		}
	}
	else if((blue_counter < red_counter) && (blue_counter < green_counter)){
		if(((red_counter-blue_counter) > YIELD_DISTINGUISH_COLOR) &&  ((green_counter-blue_counter) > YIELD_DISTINGUISH_COLOR)){
			return BLUE;
		}
	}
	else if((green_counter < blue_counter) && (green_counter < red_counter)){
		if(((blue_counter-green_counter) > YIELD_DISTINGUISH_COLOR) &&  ((red_counter-green_counter) > YIELD_DISTINGUISH_COLOR)){
			return GREEN;
		}
	}
	return WHITE;
=======
>>>>>>> 8624660db6ce97125d5ca04f847a51ed0c81d01e
}
/***************************END INTERNAL FUNCTIONS************************************/


/***************************THREADS************************************/

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
		//waits for the capture to be done
		wait_image_ready();
		//signals an image has been captured
		chBSemSignal(&image_ready_sem);
<<<<<<< HEAD
		if(retrieve_stop_camera() == true)
			chThdExit(0);
=======

>>>>>>> 8624660db6ce97125d5ca04f847a51ed0c81d01e
    }
}
uint8_t determine_goals(uint8_t *img_buff_ptr, uint8_t *image){
	uint16_t red_counter = 0 , green_counter = 0,blue_counter = 0;
	red_counter = determine_color(img_buff_ptr, image,RED);
	green_counter = determine_color(img_buff_ptr, image,GREEN);
	blue_counter = determine_color(img_buff_ptr, image,BLUE);
	//chprintf((BaseSequentialStream *)&SD3, "red : \r\n\n",red_counter);
	//chprintf((BaseSequentialStream *)&SD3, "blue : \r\n\n",blue_counter);
	//chprintf((BaseSequentialStream *)&SD3, "green : \r\n\n",green_counter);
	if((red_counter < blue_counter) && (red_counter < green_counter)){
		if(((blue_counter-red_counter) > YIELD_DISTINGUISH_COLOR) &&  ((green_counter-red_counter) > YIELD_DISTINGUISH_COLOR)){
			return RED;
		}
	}
	else if((blue_counter < red_counter) && (blue_counter < green_counter)){
		if(((red_counter-blue_counter) > YIELD_DISTINGUISH_COLOR) &&  ((green_counter-blue_counter) > YIELD_DISTINGUISH_COLOR)){
			return BLUE;
		}
	}
	else if((green_counter < blue_counter) && (green_counter < red_counter)){
		if(((blue_counter-green_counter) > YIELD_DISTINGUISH_COLOR) &&  ((red_counter-green_counter) > YIELD_DISTINGUISH_COLOR)){
			return GREEN;
		}
	}
	return WHITE;
}

<<<<<<< HEAD
/***************************EXTERNAL FUNCTIONS************************************/
//input:	 1)color to analyze
//output : returns the line width of image (goals width in image)
//purpose: allows to extract line width to use this information in a different file
uint16_t get_line_width(uint8_t color){
	static uint8_t *img_buff_ptr;
	static uint8_t image[IMAGE_BUFFER_SIZE] = {0};
    chBSemWait(&image_ready_sem);
    img_buff_ptr = dcmi_get_last_image_ptr();
	switch (color)
	{
		case RED:
			//Extracts only the RED pixels
			for(uint16_t i = 0 ; i < (2 * IMAGE_BUFFER_SIZE) ; i+=2)
				image[i/2] = (uint8_t)img_buff_ptr[i]&0xF8;
			break;
		case BLUE:
			//Extracts only the BLUE pixels
			for(uint16_t i = 0 ; i < (2 * IMAGE_BUFFER_SIZE) ; i+=2)
				image[i/2] = (((uint8_t)img_buff_ptr[i+1]&31)<<3);
			break;
		case GREEN :
			//Extracts only the GREEN pixels
			for(uint16_t i = 0 ; i < (2 * IMAGE_BUFFER_SIZE) ; i+=2)
				image[i/2] = (((uint8_t)img_buff_ptr[i+1]&224)>>3)  + (((uint8_t)img_buff_ptr[i]&3)<<5);
			break;
	}
	return extract_line_width(image);
}

//input:	 -
//output : line position is the position of the center of the goal on the picture (in pixels).
//purpose: allows to extract line position to use this information in a different file.
=======




//static THD_WORKING_AREA(waProcessImage, 1024);
//static THD_FUNCTION(ProcessImage, arg) {

    //chRegSetThreadName(__FUNCTION__);
    //(void)arg;



	//bool send_to_computer = true;

    //while(1){
    	//waits until an image has been captured

		//chprintf((BaseSequentialStream *)&SD3, "colore: %d	\r\n\n",colore);
		//chThdSleepMilliseconds(1000);
		//Extracts only the red pixels

		//search for a line in the image and gets its width in pixels

		//lineWidth = extract_line_width(image);

		//converts the width into a distance between the robot and the camera
		//if(lineWidth){
		//	distance_cm = PXTOCM/lineWidth;
		//}

		//if(send_to_computer){
			//sends to the computer the image
			//SendUint8ToComputer(image, IMAGE_BUFFER_SIZE);
		//}
		//invert the bool
		//send_to_computer = !send_to_computer;
   // }
//}

uint16_t get_line_width(uint8_t color){
	static uint8_t *img_buff_ptr;
	static uint8_t image[IMAGE_BUFFER_SIZE] = {0};
    chBSemWait(&image_ready_sem);
    img_buff_ptr = dcmi_get_last_image_ptr();
	switch (color)
		{
		 //NOTE :	VALUES AMPLIFIED (0bXXXXX000)
		    case RED:
		    		//Extracts only the RED pixels
				for(uint16_t i = 0 ; i < (2 * IMAGE_BUFFER_SIZE) ; i+=2){
				//extracts first 5bits of the first byte
				//takes nothing from the second byte
					image[i/2] = (uint8_t)img_buff_ptr[i]&0xF8;
				}
		        break;
		    case BLUE:
				//Extracts only the BLUE pixels
				for(uint16_t i = 0 ; i < (2 * IMAGE_BUFFER_SIZE) ; i+=2){
				//extracts first 5bits of the first byte
				//takes nothing from the second byte
					image[i/2] = (((uint8_t)img_buff_ptr[i+1]&31)<<3);
				}
		        break;
		    case GREEN :
				//Extracts only the GREEN pixels
				for(uint16_t i = 0 ; i < (2 * IMAGE_BUFFER_SIZE) ; i+=2){
				//extracts first 5bits of the first byte
				//takes nothing from the second byte
					image[i/2] = (((uint8_t)img_buff_ptr[i+1]&224)>>3)  + (((uint8_t)img_buff_ptr[i]&3)<<5);
				}
				break;
		}
	return extract_line_width(image);
}
>>>>>>> 8624660db6ce97125d5ca04f847a51ed0c81d01e

uint16_t get_line_position(void){
	return line_position;
}

<<<<<<< HEAD

//input:	 -
//output : color detected by camera
//purpose: allows to extract the color detected to use this information in a different file.
=======
>>>>>>> 8624660db6ce97125d5ca04f847a51ed0c81d01e
uint8_t get_color_detected(void){
	static uint8_t *img_buff_ptr;
	static uint8_t image[IMAGE_BUFFER_SIZE] = {0};
    chBSemWait(&image_ready_sem);
	//gets the pointer to the array filled with the last image in RGB565
	img_buff_ptr = dcmi_get_last_image_ptr();
	color_detected = determine_goals(img_buff_ptr,image);
<<<<<<< HEAD
	return color_detected;
}

//input:	 -
//output : -
//purpose: initializes threads in process image
void process_image_start(void){
=======
	//chprintf((BaseSequentialStream *)&SD3, "color:  %d\r\n\n",color_detected);
	return color_detected;
}

void process_image_start(void){
	//chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);
>>>>>>> 8624660db6ce97125d5ca04f847a51ed0c81d01e
	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);
}
