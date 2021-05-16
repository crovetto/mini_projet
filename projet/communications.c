#include "ch.h"
#include "hal.h"
#include <main.h>

#include <communications.h>

/*
*	Sends floats numbers to the computer
*/
void SendFloatToComputer(BaseSequentialStream* out, float* data, uint16_t size) 
{	
	chSequentialStreamWrite(out, (uint8_t*)"START", 5);
	chSequentialStreamWrite(out, (uint8_t*)&size, sizeof(uint16_t));
	chSequentialStreamWrite(out, (uint8_t*)data, sizeof(float) * size);
}

