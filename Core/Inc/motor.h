/*
 * motor.h
 *
 *  Created on: Feb 7, 2024
 *      Author: Edwin
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

/* motor rotation directions */
#define CW 0
#define CCW 1

/* motor pwm properties */
#define MOTOR_PWM_RES	10
#define MOTOR_F_PWM		500

/* number of motors to be used */
#define MOTOR_UNITS 1

typedef struct {
	GPIO_TypeDef* IN1_GPIO;
	GPIO_TypeDef* IN2_GPIO;
	uint16_t	  IN1_PIN;
	uint16_t	  IN2_PIN;
	TIM_TypeDef*  TIM_Instance;
	uint32_t 	  PWM_TIM_CH;
	uint32_t	  TIM_CLK_MHz;
	uint32_t	  PWM_FREQ_Hz;
	uint8_t		  PWM_RES_BITS;

}  MOTOR_ConfigTypeDef;


void motor_init(uint8_t motor_instance);
void motor_start(uint8_t motor_instance, uint8_t dir, uint8_t speed);
void motor_set_speed(uint8_t motor_instance, uint16_t speed);
void motor_set_dir(uint8_t motor_instance, uint8_t dir);
void motor_stop(uint8_t motor_instance);
uint32_t motor_get_max_freq(uint8_t motor_instance);

/**
 * Move motor forward
 */
void moveForward();

/**
 * Move motor reverse
 */
void reverse();

/**
 * Stop motors
 */
void stop();

#endif /* INC_MOTOR_H_ */
