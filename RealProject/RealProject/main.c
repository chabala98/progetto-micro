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
#include "audio/audio_thread.h"


#include "cmd.h"
#include "config_flash_storage.h"
#include "exti.h"
#include "i2c_bus.h"
#include <main.h>

#include <robotmvmts.h>
#include <process_image.h>
#include "sensors/VL53L0X/VL53L0X.h"
#include "audio/play_melody.h"
#include "audio/play_sound_file.h"


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
	dac_start();
	//inits the motors
	clear_leds();

	motors_init();
	VL53L0X_start();


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



#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
