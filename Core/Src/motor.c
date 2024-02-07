/*
 * motor.c
 *
 *  Created on: Feb 7, 2024
 *      Author: Edwin
 */

#include "motor.h"

/**
 * Move motor forward
 */
void moveForward() {
	HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, 1);
	HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, 0);

	HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, 1);
	HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, 0);
}

/**
 * Move motor reverse
 */
void reverse() {

	HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, 0);
	HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, 1);

	HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, 0);
	HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, 1);

}

/**
 * Stop motors
 */
void stop() {

	HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, 1);
	HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, 1);

	HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, 1);
	HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, 1);

}
