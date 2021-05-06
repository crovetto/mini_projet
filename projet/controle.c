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

#define COTE_MAX				500
#define TRIANGLE_ARETE_MAX	3
#define ANGLE_MAX			COTE_MAX+450
#define 	ANGLE_MIN			COTE_MAX-450


static uint8_t nb_arete=0;

bool obstacle_detection(void){
	if(VL53L0X_get_dist_mm()<50)
	{
		right_motor_set_speed(0);
		left_motor_set_speed(0);
		return true;
	}
	else{
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

//		if(nb_arete==0 || nb_arete==1 || nb_arete==2 || nb_arete==3){
//			left_motor_set_pos(0);
//			right_motor_set_pos(0);
//		}

		if(!obstacle_detection()){
			set_body_led(0);
			switch(get_forme())
			{
				case 1:
					if(nb_arete<TRIANGLE_ARETE_MAX){
						if(right_motor_get_pos()<COTE_MAX && left_motor_get_pos()<COTE_MAX){
							clear_not_moving();
							right_motor_set_speed(600);
							left_motor_set_speed(600);
						}
						else if(nb_arete<TRIANGLE_ARETE_MAX-1 && right_motor_get_pos()<ANGLE_MAX && left_motor_get_pos()>ANGLE_MIN){
							right_motor_set_speed(600);
							left_motor_set_speed(-600);
						}
						else{
							nb_arete++;
							right_motor_set_pos(0);
							left_motor_set_pos(0);
						}

					}
					else {
						nb_arete=0;
						right_motor_set_speed(0);
						left_motor_set_speed(0);
						right_motor_set_pos(0);
						left_motor_set_pos(0);
						set_not_moving();
						clear_forme();
						set_front_led(1);
					}
					break;

				default:
					nb_arete=0;
					right_motor_set_pos(0);
					left_motor_set_pos(0);
					right_motor_set_speed(0);
					left_motor_set_speed(0);
					set_not_moving();
					clear_forme();
					break;
			}
		}
		else{
			set_body_led(1);
		}



		//			case TRIANGLE:
			//					//	CONTROLE DISTANCE DETECTION
			//				if(right_motor_get_pos()<COTE_MAX && left_motor_get_pos()<COTE_MAX){
			//					right_motor_set_speed(600);
			//					left_motor_set_speed(600);
			//
			//				}
			//
			//				else if(nb_arete<TRIANGLE_ARETE_MAX && right_motor_get_pos()<=ANGLE_MAX && left_motor_get_pos()>=ANGLE_MIN){
			//					right_motor_set_speed(600);
			//					left_motor_set_speed(-600);
			//				}
			//				else if(right_motor_get_pos()==ANGLE_MAX && left_motor_get_pos()==ANGLE_MIN){
			//						nb_arete++;
			//
			//				}
			//				else{
			//					right_motor_set_speed(0);
			//					left_motor_set_speed(0);
			//				}
			//				break;
			//			default:
			//				break;




//		int n=0;
//		switch (get_forme())
//				{
//					case 1:
//						//avance
//						n= 10500000*6; //speed
//						right_motor_set_speed(600);
//						left_motor_set_speed(600);
//						while (n--){
//							__asm__ volatile("nop");
//						}
//						//tourne
//						right_motor_set_speed(600);
//						left_motor_set_speed(-600);
//						n= 10500000*3;
//						while (n--){
//							__asm__ volatile("nop");
//						}
//						//avance
//						n= 10500000*6; //speed
//						right_motor_set_speed(600);
//						left_motor_set_speed(600);
//						while (n--){
//							__asm__ volatile("nop");
//						}
//						//tourne
//						right_motor_set_speed(600);
//						left_motor_set_speed(-600);
//						n= 10500000*3;
//						while (n--){
//							__asm__ volatile("nop");
//						}
//						//avance
//						n= 10500000*6; //speed
//						right_motor_set_speed(600);
//						left_motor_set_speed(600);
//						while (n--){
//							__asm__ volatile("nop");
//						}
//						//stop
//						right_motor_set_speed(0);
//						left_motor_set_speed(0);
//
//						clear_forme();
//
//						break;
//					case CARRE:
//						n= 10500000*8; //speed
//						right_motor_set_speed(-600);
//						left_motor_set_speed(-600);
//						while (n--){
//							__asm__ volatile("nop");
//						}
//						//stop
//						right_motor_set_speed(0);
//						left_motor_set_speed(0);
//						break;
//					default:
//						break;
//				}

		//100Hz
		chThdSleepUntilWindowed(time, time + MS2ST(10));
	}

}


void dessin_start(void){
	chThdCreateStatic(waDessin, sizeof(waDessin), NORMALPRIO+10, Dessin, NULL);
}





