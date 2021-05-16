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
#include <controle.h>

#define DIST_OBSTACLE			80 	//en mm
#define STOP					0
#define GO 						400 // Vitesse des moteurs step/s : 15.08 cm/s

#define PI                 		3.1415926536f

#define NB_STEP_ONE_TURN		1000 // Nombre de step pour un tour du moteur
#define WHEEL_PERIMETER			37.7f

#define	CM_TO_STEP(x)			x *	(NB_STEP_ONE_TURN/WHEEL_PERIMETER)

#define WHEEL_DISTANCE      	8    //Ecart des roues en cm
#define PERIMETER_EPUCK         PI * WHEEL_DISTANCE //en cm

#define LONGUEUR				CM_TO_STEP(12) 	//Longueur des cotes des formes : 12 cm

//TRIANGLE
#define T_ARETE_MAX				3
#define T_ANGLE_MAX				LONGUEUR + CM_TO_STEP(PERIMETER_EPUCK/2.78f) // longueur + angle supplémentaire de 60 degré en step
#define T_ANGLE_MIN				LONGUEUR - CM_TO_STEP(PERIMETER_EPUCK/2.78f) //longueur - angle supplémentaire de 60 degré en step
//CARRE
#define C_ARETE_MAX				4
#define C_ANGLE_MAX				LONGUEUR + CM_TO_STEP(PERIMETER_EPUCK/3.665f) //longueur + angle supplémentaire de 90 degré en step
#define C_ANGLE_MIN				LONGUEUR - CM_TO_STEP(PERIMETER_EPUCK/3.66f)//longueur - angle supplémentaire de 90 degré en step
//COURBES
#define LONGUEUR_COURBE			2 * (NB_STEP_ONE_TURN/WHEEL_PERIMETER) 	// 2 cm
#define COURBE_ARETE_MAX		5
#define COURBE_ANGLE_MAX		LONGUEUR_COURBE+37
#define COURBE_ANGLE_MIN		LONGUEUR_COURBE-37

#define TIME_RESET					300 // temps : 3s

static uint8_t nb_arete=0;
static uint16_t count_pause=0;

/**************************MOTORS***********************************/

void go_forward(uint16_t speed){
    right_motor_set_speed(speed);
    left_motor_set_speed(speed);
}

void turn_left(uint16_t speed){
    right_motor_set_speed(speed);
    left_motor_set_speed(-speed);
}

void turn_right(uint16_t speed){
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
	if(VL53L0X_get_dist_mm()<DIST_OBSTACLE)
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


void go_triangle(void){
	if(nb_arete<T_ARETE_MAX){
		if(right_motor_get_pos()<LONGUEUR && left_motor_get_pos()<LONGUEUR){
			go_forward(GO);
		}
		else if(nb_arete<T_ARETE_MAX-1 && right_motor_get_pos()<T_ANGLE_MAX && left_motor_get_pos()>T_ANGLE_MIN){
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
}

void go_square(void){
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
}

void go_right(void){
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
}

void go_left(void){
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
					go_triangle();
					break;

				case CARRE:
					go_square();
					break;

				case DROITE:
					go_right();
					break;
				case GAUCHE:
					go_left();
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

