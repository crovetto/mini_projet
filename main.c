//MINI_PROJET
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <leds.h>
#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <main.h>
#include <sensors/VL53L0X/VL53L0X.h>

#include <chprintf.h>
#include <usbcfg.h>
#include <motors.h>


#include <audio/microphone.h>
#include <audio_processing.h>
#include <fft.h>
#include <communications.h>
#include <arm_math.h>
#include <controle.h>

//uncomment to send the FFTs results from the real microphones
#define SEND_FROM_MIC

//uncomment to use double buffering to send the FFT to the computer
#define DOUBLE_BUFFERING

void SendUint8ToComputer(uint8_t* data, uint16_t size)
{
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)"START", 5);
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)&size, sizeof(uint16_t));
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)data, size);
}

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



static void timer11_start(void){
    //General Purpose Timer configuration
    //timer 11 is a 16 bit timer so we can measure time
    //to about 65ms with a 1Mhz counter
    static const GPTConfig gpt11cfg = {
        1000000,        /* 1MHz timer clock in order to measure uS.*/
        NULL,           /* Timer callback.*/
        0,
        0
    };

    gptStart(&GPTD11, &gpt11cfg);
    //let the timer count to max value
    gptStartContinuous(&GPTD11, 0xFFFF);
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
    //starts timer 11
    timer11_start();
    	//inits the motors
    	motors_init();
    	//start TOF
    	VL53L0X_start();




    	//temp tab used to store values in complex_float format
	//needed bx doFFT_c
//	static complex_float temp_tab[FFT_SIZE];
	//send_tab is used to save the state of the buffer to send (double buffering)
	//to avoid modifications of the buffer while sending it
	static float send_tab[FFT_SIZE];


	// A enlever
	#ifdef SEND_FROM_MIC
		//starts the microphones processing thread.
		//it calls the callback given in parameter when samples are ready
		mic_start(&processAudioData);
	#endif  /* SEND_FROM_MIC */



    /* Infinite loop. */
    while (1) {

    	/*A enlever ifdef ? */
	//#ifdef SEND_FROM_MIC
			//waits until a result must be sent to the computer
			wait_send_to_computer();

			/*A enlever ifdef ? */
	//#ifdef DOUBLE_BUFFERING
			//we copy the buffer to avoid conflicts
			arm_copy_f32(get_audio_buffer_ptr(LEFT_OUTPUT), send_tab, FFT_SIZE);
			SendFloatToComputer((BaseSequentialStream *) &SD3, send_tab, FFT_SIZE);   // Nécessaire ?



    	//waits 1 second
    //chThdSleepMilliseconds(1000);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
