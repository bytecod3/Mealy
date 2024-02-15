/*
 * motor_config.c
 *
 *  Created on: Feb 7, 2024
 *      Author: USER
 */


#include "motor.h"

const MOTOR_ConfigTypeDef motor_config_params[MOTOR_UNITS] =
{
		// dc motor 1 configurations
		{
				GPIOB,			// in1 port
				GPIOB,			// in2 port
				GPIO_PIN_1,		// in1 pin
				GPIO_PIN_13,	// in2 pin
				TIM2,			// timer used
				TIM_CHANNEL_1,	// timer channel
				8,				// clock freq (MHz)
				MOTOR_F_PWM,	// pwm period
				MOTOR_PWM_RES	// pwm resolution

		},

		// dc motor 2 configurations
		{
				GPIOB,			// in1 port
				GPIOB,			// in2 port
				GPIO_PIN_14,	// in1 pin
				GPIO_PIN_15,	// in2 pin
				TIM2,			// timer used
				TIM_CHANNEL_2,	// timer channel
				8,				// clock freq (MHz)
				MOTOR_F_PWM,	// pwm period
				MOTOR_PWM_RES	// pwm resolution

		}

};
