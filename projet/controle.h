/*
 * controle.h
 *
 *  Created on: 5 May 2021
 *      Author: giannacrovetto
 */

#ifndef CONTROLE_H_
#define CONTROLE_H_

 /**
 * @brief   Set the same speed to the motors.
 */
void go_forward(uint16_t speed);

 /**
 * @brief   Set the speed to the motors to turn left.
 */
void turn_left(uint16_t speed);

 /**
 * @brief   Set the speed to the motors to turn right.
 */
void turn_right(uint16_t speed);

/**
 * @brief 	sets the position counter of both motors to zero
 * 
 */
void clear_pos(void);

/*
*	Global reset 
*/
void reset(void);

/*
*	Detect an obstacle and stop or enable movement 
*	Reset the robot if the obstacle stays longer than 3s
*/
bool obstacle_detection(void);

/**
* @brief   Do a triangle.
*/
void go_triangle(void);
/**
* @brief   Do a square.
*/
void go_square(void);
/**
* @brief   Go right.
*/
void go_right(void);
/**
* @brief   go left.
*/
void go_left(void);

/*
*	Enable thread
*/
void dessin_start(void);


#endif /* CONTROLE_H_ */
