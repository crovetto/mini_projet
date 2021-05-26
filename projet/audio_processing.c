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

#include <leds.h>

//semaphore
static BSEMAPHORE_DECL(sendToComputer_sem, TRUE);

//2 times FFT_SIZE because these arrays contain complex numbers (real + imaginary)
													/*Les variables statiques gloabales suivantes commentees sont inutilisees dans notre cas.
													 * On pourrait les supprimer (si on decide de réduire la fonction get_audio_buffer_ptr().
													 * Si nous gardons la fonction en l'etat (avec sa modularite/globalite),
													 * nous aurions du les declarer dans la fonction get_audio_buffer_ptr()
													 * pour des questions d'optimisation et de gestion de memoire.
													 */
static float micLeft_cmplx_input[2 * FFT_SIZE];			//inutilisee
static float micRight_cmplx_input[2 * FFT_SIZE];		//inutilisee
static float micFront_cmplx_input[2 * FFT_SIZE];		//inutilisee
static float micBack_cmplx_input[2 * FFT_SIZE];
//Arrays containing the computed magnitude of the complex numbers
static float micLeft_output[FFT_SIZE];					//inutilisee
static float micRight_output[FFT_SIZE];					//inutilisee
static float micFront_output[FFT_SIZE];					//inutilisee
static float micBack_output[FFT_SIZE];

#define MIN_VALUE_THRESHOLD	10000 

#define MIN_FREQ			10	//we don't analyze before this index to not use resources for nothing
#define FREQ_TRIANGLE		21	//328Hz
#define FREQ_CARRE			24	//359Hz
#define FREQ_RIGHT			31	//480HZ
#define FREQ_LEFT			35	//540Hz
#define MAX_FREQ			40	//we don't analyze after this index to not use resources for nothing

#define FREQ_TRIANGLE_L			(FREQ_TRIANGLE-0.1f)
#define FREQ_TRIANGLE_H			(FREQ_TRIANGLE+0.1f)
#define FREQ_CARRE_L			(FREQ_CARRE-0.1f)
#define FREQ_CARRE_H			(FREQ_CARRE+0.1f)
#define FREQ_RIGHT_L			(FREQ_RIGHT-0.1f)
#define FREQ_RIGHT_H			(FREQ_RIGHT+0.1f)
#define FREQ_LEFT_L				(FREQ_LEFT-0.1f)
#define FREQ_LEFT_H				(FREQ_LEFT+0.1f)

static uint8_t moving=0;
static uint8_t shape = 0;


/*
*	Simple fonction utilisee pour detecter la plus grande valeur dans un buffer
*	et selon la frequence, retourne une forme specifique
*/
uint8_t son_detection(float* data){
	float max_norm = MIN_VALUE_THRESHOLD;
	int16_t max_norm_index = -1;

	//search for the highest peak
	for(uint16_t i = MIN_FREQ ; i <= MAX_FREQ ; i++){
		if(data[i] > max_norm){
			max_norm = data[i];
			max_norm_index = i;
		}
	}
	//TRIANGLE
	if(max_norm_index >= FREQ_TRIANGLE_L && max_norm_index <= FREQ_TRIANGLE_H){
		return TRIANGLE;

	}
	//CARRE
	else if(max_norm_index >= FREQ_CARRE_L && max_norm_index <= FREQ_CARRE_H){
		return CARRE;
	}
	//courbe droite
	else if(max_norm_index >= FREQ_RIGHT_L && max_norm_index <= FREQ_RIGHT_H){
		return DROITE;
	}
	//courbe gauche
	else if(max_norm_index >= FREQ_LEFT_L && max_norm_index <= FREQ_LEFT_H){
	return GAUCHE;
	}
	else{
		return 0;
	}

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
	*	We get 160 samples from the back mic every 10ms
	*	So we fill the samples buffers to reach
	*	1024 samples, then we compute the FFTs.
	*
	*/
	static uint16_t nb_samples = 0;
	static uint8_t mustSend = 0;

	//loop to fill the buffers
	for(uint16_t i = 0 ; i < num_samples ; i+=4){
		//construct an array of complex numbers. Put 0 to the imaginary part
		micBack_cmplx_input[nb_samples] = (float)data[i + MIC_BACK];

		nb_samples++;

		micBack_cmplx_input[nb_samples] = 0;

		nb_samples++;

		//stop when buffer is full
		if(nb_samples >= (2 * FFT_SIZE)){
			break;
		}
	}

	if(nb_samples >= (2 * FFT_SIZE)){
		/*	FFT proccessing
		*
		*	This FFT function stores the results in the input buffer given.
		*	This is an "In Place" function. 
		*/
		doFFT_optimized(FFT_SIZE, micBack_cmplx_input);

		/*	Magnitude processing
		*
		*	Computes the magnitude of the complex numbers and
		*	stores them in a buffer of FFT_SIZE because it only contains
		*	real numbers.
		*
		*/
		arm_cmplx_mag_f32(micBack_cmplx_input, micBack_output, FFT_SIZE);

		//sends only one FFT result over 10 for 1 mic to not flood the computer
		//sends to UART3
		if(mustSend > 8){
			//signals to send the result to the computer
			chBSemSignal(&sendToComputer_sem);
			mustSend = 0;
		}
		nb_samples = 0;
		mustSend++;


		if(!moving){
			shape = son_detection(micBack_output);
			set_moving();
		}
	}
}


void set_moving(void)
{
	moving=1;
}
void clear_moving(void)
{
	moving=0;
}

uint8_t get_shape(void)
{
	return shape;
}

void clear_shape(void)
{
	shape=0;
}

void wait_send_to_computer(void){
	chBSemWait(&sendToComputer_sem);
}

/* Cette fonction aurait due etre simplifiee (en supprimant les parties commentees ci-dessous)
 * pour pouvoir enlever les nombreuses variables statiques declarees en debut de fichier, qui sont inutilisees.
 * Elle est redondante en l'etat. C'est une fonction qui est plus globale et modulaire que ce qu'on en fait.
 */

float* get_audio_buffer_ptr(BUFFER_NAME_t name){
	if(name == LEFT_CMPLX_INPUT){			//d'ici
		return micLeft_cmplx_input;
	}
	else if (name == RIGHT_CMPLX_INPUT){
		return micRight_cmplx_input;
	}
	else if (name == FRONT_CMPLX_INPUT){
		return micFront_cmplx_input;
	}
	else if (name == BACK_CMPLX_INPUT){		//sauf lui
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
	}										//jusque la
	else if (name == BACK_OUTPUT){			//en remplacant ce "else if" en "if"
		return micBack_output;
	}
	else{
		return NULL;
	}
}
