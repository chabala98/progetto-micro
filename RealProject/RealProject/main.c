#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>


#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h>
#include <main.h>
#include <motors.h>
//#include <camera/po8030.h>
#include <chprintf.h>
#include <audio/microphone.h>
#include <audio_processing.h>
#include <fft.h>
#include <arm_math.h>
#include <spi_comm.h>
#include <leds.h>
<<<<<<< HEAD
#include "audio/audio_thread.h"
=======
>>>>>>> 8624660db6ce97125d5ca04f847a51ed0c81d01e


#include "cmd.h"
#include "config_flash_storage.h"
#include "exti.h"
#include "i2c_bus.h"
#include <main.h>
<<<<<<< HEAD

#include <robotmvmts.h>
#include <process_image.h>
#include "sensors/VL53L0X/VL53L0X.h"
#include "audio/play_melody.h"
#include "audio/play_sound_file.h"

=======
//#include "communication.h"

#include <robotmvmts.h>
#include <process_image.h>
#include "sensors/VL53L0X/VL53L0X.h"

//uncomment to send the FFTs results from the real microphones
#define SEND_FROM_MIC

//uncomment to use double buffering to send the FFT to the computer
//#define DOUBLE_BUFFERING
>>>>>>> 8624660db6ce97125d5ca04f847a51ed0c81d01e

static void serial_start(void)
{
	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};

	sdStart(&SD3, &ser_cfg); // UART3.
}


void SendUint8ToComputer(uint8_t* data, uint16_t size) 
{
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)"START", 5);
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)&size, sizeof(uint16_t));
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)data, size);
}
<<<<<<< HEAD
=======


>>>>>>> 8624660db6ce97125d5ca04f847a51ed0c81d01e


int main(void)
{

    halInit();
    chSysInit();
    mpu_init();

    //starts the serial communication
    serial_start();
    //start the USB communication
    usb_start();
    //starts the camera
    dcmi_start();
	po8030_start();
	spi_comm_start();
<<<<<<< HEAD
	dac_start();
=======
>>>>>>> 8624660db6ce97125d5ca04f847a51ed0c81d01e
	//inits the motors
	clear_leds();

	motors_init();
	VL53L0X_start();
	//init_identify_world();

	process_image_start();
	chThdSleepMilliseconds(5000);
	robotmvmts_start();
	playMelodyStart();
	playSoundFileStart();


	/* IN THIS SITUATION, IT WORKS
	 chThdSleepMilliseconds(10);
	 playMelody(IM_BLUE, ML_FORCE_CHANGE,NULL);
	 playMelody(CALIFORNICATION, ML_FORCE_CHANGE,NULL);
	 playMelody(IM_BLUE, ML_FORCE_CHANGE,NULL);
	 chThdSleepMilliseconds(10000);
	 playMelody(BROKEN_DREAMS, ML_FORCE_CHANGE,NULL);
	 */

	/* IN THIS SITUATION, IT DOESN'T WORK
	 chThdSleepMilliseconds(10);
	 playMelody(IM_BLUE, ML_FORCE_CHANGE,NULL);
	 playMelody(CALIFORNICATION, ML_FORCE_CHANGE,NULL);
	 playMelody(BROKEN_DREAM, ML_FORCE_CHANGE,NULL);
	 chThdSleepMilliseconds(10000);
	 playMelody(IM_BLUE, ML_FORCE_CHANGE,NULL);
	 */

   while (1) {
	   chThdSleepMilliseconds(500);
   }
}

<<<<<<< HEAD

=======
	//stars the threads for the pi regulator and the processing of the image
	//pi_regulator_start();
	process_image_start();
	robotmvmts_start();

    //temp tab used to store values in complex_float format
    //needed bx doFFT_c

    //send_tab is used to save the state of the buffer to send (double buffering)
    //to avoid modifications of the buffer while sending it


    /* Infinite loop. */
   while (1) {
    		chThdSleepMilliseconds(1000);
   }
}
>>>>>>> 8624660db6ce97125d5ca04f847a51ed0c81d01e



#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
