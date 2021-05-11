/*
 * controle.c
 *
 *  Created on: 5 May 2021
 *      Author: giannacrovetto
 */
#include "ch.h"
#include "hal.h"
#include <sensors/VL53L0X/VL53L0X.h>
#include "motors.h"
#include <audio_processing.h>
#include <stdbool.h>

#include <leds.h>

#define STOP					0
#define GO 						600

#define LONGUEUR				500 //longueur des cotes des formes
//TRIANGLE
#define T_ARETE_MAX				3
#define T_ANGLE_MAX				LONGUEUR+450
#define T_ANGLE_MIN				LONGUEUR-450
//CARRE
#define C_ARETE_MAX				4
#define C_ANGLE_MAX				LONGUEUR+340
#define C_ANGLE_MIN				LONGUEUR-340
//COURBES
#define LONGUEUR_COURBE			200
#define COURBE_ARETE_MAX		5
#define COURBE_ANGLE_MAX		LONGUEUR_COURBE+37
#define COURBE_ANGLE_MIN		LONGUEUR_COURBE-37

#define TIME_RESET					300 // si l'obstacle reste 3s, le robot s'arrête

static uint8_t nb_arete=0;
static uint16_t count_pause=0;

/**************************MOTORS***********************************/

void go_forward(uint8_t speed){
    right_motor_set_speed(speed);
    left_motor_set_speed(speed);
}

void turn_left(uint8_t speed){
    right_motor_set_speed(speed);
    left_motor_set_speed(-speed);
}

void turn_right(uint8_t speed){
    right_motor_set_speed(-speed);
    left_motor_set_speed(speed);
}

void clear_pos(void){
    right_motor_set_pos(0);
    left_motor_set_pos(0);
}

/**************************END MOTORS***********************************/

void reset(void){
	nb_arete=0;
	clear_pos();
	go_forward(STOP);
	clear_moving();
	clear_shape();
}

bool obstacle_detection(void){
	if(VL53L0X_get_dist_mm()<50)
	{
		set_body_led(1);
		go_forward(STOP);

		count_pause++;
		if(count_pause >= TIME_RESET) // l'obstacle reste 3s : reset du robot
		{
			reset();
			set_body_led(0);
		}
		return true;
	}
	else{
		set_body_led(0);
		count_pause = 0;
		return false;
	}
}



static THD_WORKING_AREA(waDessin,256);
static THD_FUNCTION(Dessin,arg){
	(void) arg;
	chRegSetThreadName(__FUNCTION__);

	systime_t time;

	while(1){
		time = chVTGetSystemTime();

		if(!obstacle_detection()){
			switch(get_shape())
			{
				case TRIANGLE:
					if(nb_arete<T_ARETE_MAX){
						if(right_motor_get_pos()<LONGUEUR && left_motor_get_pos()<LONGUEUR){
							go_forward(GO);
						}
						else if(nb_arete<T_ARETE_MAX-1 && right_motor_get_pos()<T_ANGLE_MAX && left_motor_get_pos()>T_ANGLE_MIN){
							turn_left(GO);
						}
						else{
							nb_arete++;
							go_forward(STOP);
						}
					}
					else {
						reset();

					}
					break;

				case CARRE:
					if(nb_arete<C_ARETE_MAX){
						if(right_motor_get_pos()<LONGUEUR && left_motor_get_pos()<LONGUEUR){
							go_forward(GO);
						}
						else if(nb_arete < C_ARETE_MAX-1 && right_motor_get_pos() < C_ANGLE_MAX && left_motor_get_pos() > C_ANGLE_MIN){
							turn_left(GO);
						}
						else{
							nb_arete++;
							clear_pos();
						}
					}
					else {
						reset();
					}
					break;

				case DROITE:
					if(nb_arete<COURBE_ARETE_MAX){
						if(right_motor_get_pos()<LONGUEUR_COURBE && left_motor_get_pos()<LONGUEUR_COURBE){
							go_forward(GO);
						}
						else if(nb_arete < COURBE_ARETE_MAX-1 && right_motor_get_pos() > COURBE_ANGLE_MIN && left_motor_get_pos() < COURBE_ANGLE_MAX){
							turn_right(GO);
						}
						else{
							nb_arete++;
							clear_pos();
						}
					}
					else {
						reset();
					}
					break;
				case GAUCHE:
					if(nb_arete<COURBE_ARETE_MAX){
						if(right_motor_get_pos()<LONGUEUR_COURBE && left_motor_get_pos()<LONGUEUR_COURBE){
							go_forward(GO);
						}
						else if(nb_arete < COURBE_ARETE_MAX-1 && right_motor_get_pos() < COURBE_ANGLE_MAX && left_motor_get_pos() > COURBE_ANGLE_MIN ){
							turn_left(GO);
						}
						else{
							nb_arete++;
							clear_pos();
						}
					}
					else {
						reset();
					}
					break;

				default:
					reset();
					break;
			}
		}

		//100Hz
		chThdSleepUntilWindowed(time, time + MS2ST(10));
	}

}


void dessin_start(void){
	chThdCreateStatic(waDessin, sizeof(waDessin), NORMALPRIO+10, Dessin, NULL);
}




