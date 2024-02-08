/*
 * motor.c
 *
 *  Created on: Feb 7, 2024
 *      Author: Edwin
 */

#include "motor.h"
#include "motor_config.h"
#include "stm32f1xx_hal.h"


void motor_init(uint8_t motor_instance) {

	GPIO_InitTypeDef GPIO_InitStruct = {0};
	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};
	TIM_HandleTypeDef htim;

	uint32_t PSC_Value = 0;
	uint32_t ARR_Value = 0;
	uint8_t i = 0;

	/* configure the 2 direction control pins */
	if(motor_config_params[motor_instance].IN1_GPIO == GPIOA || motor_config_params[motor_instance].IN2_GPIO == GPIOA)
	{
		__HAL_RCC_GPIOA_CLK_ENABLE();
	}
	else if(motor_config_params[motor_instance].IN1_GPIO == GPIOB || motor_config_params[motor_instance].IN2_GPIO == GPIOB)
	{
		__HAL_RCC_GPIOB_CLK_ENABLE();
	}
	else if(motor_config_params[motor_instance].IN1_GPIO == GPIOC || motor_config_params[motor_instance].IN2_GPIO == GPIOC)
	{
		__HAL_RCC_GPIOC_CLK_ENABLE();
	}
	else if(motor_config_params[motor_instance].IN1_GPIO == GPIOD || motor_config_params[motor_instance].IN2_GPIO == GPIOD)
	{
		__HAL_RCC_GPIOD_CLK_ENABLE();
	}
	else if(motor_config_params[motor_instance].IN1_GPIO == GPIOE || motor_config_params[motor_instance].IN2_GPIO == GPIOE)
	{
		__HAL_RCC_GPIOE_CLK_ENABLE();
	}

	GPIO_InitStruct.Pin = motor_config_params[motor_instance].IN1_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(motor_config_params[motor_instance].IN1_GPIO, &GPIO_InitStruct);
	GPIO_InitStruct.Pin = motor_config_params[motor_instance].IN2_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(motor_config_params[motor_instance].IN2_GPIO, &GPIO_InitStruct);
	HAL_GPIO_WritePin(motor_config_params[motor_instance].IN1_GPIO, motor_config_params[motor_instance].IN1_PIN, 0);
	HAL_GPIO_WritePin(motor_config_params[motor_instance].IN2_GPIO, motor_config_params[motor_instance].IN2_PIN, 0);


	/* calculate the PSC and ARR values */
	ARR_Value = 1;
	for(i = 0; i < motor_config_params[motor_instance].PWM_RES_BITS; i++) {
		ARR_Value *= 2;
	}

	PSC_Value = (uint32_t) ((motor_config_params[motor_instance].TIM_CLK_MHz*1000000) / (ARR_Value*motor_config_params[motor_instance].PWM_FREQ_Hz) );
	PSC_Value--;
	ARR_Value -= 2;

	/* configure the DC motor PWM timer channel */
	htim.Instance = motor_config_params[motor_instance].TIM_Instance;
	htim.Init.Prescaler = PSC_Value;
	htim.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim.Init.Period = ARR_Value;
	htim.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	HAL_TIM_Base_Init(&htim);
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	HAL_TIM_ConfigClockSource(&htim, &sClockSourceConfig);
	HAL_TIM_PWM_Init(&htim);
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	HAL_TIMEx_MasterConfigSynchronization(&htim, &sMasterConfig);
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	HAL_TIM_PWM_ConfigChannel(&htim, &sConfigOC, motor_config_params[motor_instance].PWM_TIM_CH);
	HAL_TIM_MspPostInit(&htim);

	/* start the PWM channel */
	HAL_TIM_PWM_Start(&htim, motor_config_params[motor_instance].PWM_TIM_CH);



}

void motor_start(uint8_t motor_instance, uint8_t dir, uint8_t speed) {

	/* write to the two direction control pins */
	if(dir == CW) {
		HAL_GPIO_WritePin(motor_config_params[motor_instance].IN1_GPIO, motor_config_params[motor_instance].IN1_PIN, 1);
		HAL_GPIO_WritePin(motor_config_params[motor_instance].IN2_GPIO, motor_config_params[motor_instance].IN2_PIN, 0);
	} else if (dir == CCW) {
		HAL_GPIO_WritePin(motor_config_params[motor_instance].IN1_GPIO, motor_config_params[motor_instance].IN1_PIN, 0);
		HAL_GPIO_WritePin(motor_config_params[motor_instance].IN2_GPIO, motor_config_params[motor_instance].IN2_PIN, 1);
	}

	/* write speed to the PWM CH Duty Cycle register */
	if(motor_config_params[motor_instance].PWM_TIM_CH == TIM_CHANNEL_1)
	{
		motor_config_params[motor_instance].TIM_Instance->CCR1 = speed;
	}
	else if(motor_config_params[motor_instance].PWM_TIM_CH == TIM_CHANNEL_2)
	{
		motor_config_params[motor_instance].TIM_Instance->CCR2 = speed;
	}
	else if(motor_config_params[motor_instance].PWM_TIM_CH == TIM_CHANNEL_3)
	{
		motor_config_params[motor_instance].TIM_Instance->CCR3 = speed;
	}
	else
	{
		motor_config_params[motor_instance].TIM_Instance->CCR4 = speed;
	}

}

void motor_set_speed(uint8_t motor_instance, uint16_t speed) {

	/* write speed to the PWM CH Duty Cycle register */
	if(motor_config_params[motor_instance].PWM_TIM_CH == TIM_CHANNEL_1)
	{
		motor_config_params[motor_instance].TIM_Instance->CCR1 = speed;
	}
	else if(motor_config_params[motor_instance].PWM_TIM_CH == TIM_CHANNEL_2)
	{
		motor_config_params[motor_instance].TIM_Instance->CCR2 = speed;
	}
	else if(motor_config_params[motor_instance].PWM_TIM_CH == TIM_CHANNEL_3)
	{
		motor_config_params[motor_instance].TIM_Instance->CCR3 = speed;
	}
	else
	{
		motor_config_params[motor_instance].TIM_Instance->CCR4 = speed;
	}

}

void motor_set_dir(uint8_t motor_instance, uint8_t dir) {
	/* write to the two direction control pins */
	if(dir == CW) {
		HAL_GPIO_WritePin(motor_config_params[motor_instance].IN1_GPIO, motor_config_params[motor_instance].IN1_PIN, 1);
		HAL_GPIO_WritePin(motor_config_params[motor_instance].IN2_GPIO, motor_config_params[motor_instance].IN2_PIN, 0);
	} else if (dir == CCW) {
		HAL_GPIO_WritePin(motor_config_params[motor_instance].IN1_GPIO, motor_config_params[motor_instance].IN1_PIN, 0);
		HAL_GPIO_WritePin(motor_config_params[motor_instance].IN2_GPIO, motor_config_params[motor_instance].IN2_PIN, 1);
	}

}

void motor_stop(uint8_t motor_instance) {

	HAL_GPIO_WritePin(motor_config_params[motor_instance].IN1_GPIO, motor_config_params[motor_instance].IN1_PIN, 0);
	HAL_GPIO_WritePin(motor_config_params[motor_instance].IN2_GPIO, motor_config_params[motor_instance].IN2_PIN, 0);

	if(motor_config_params[motor_instance].PWM_TIM_CH == TIM_CHANNEL_1)
	{
		motor_config_params[motor_instance].TIM_Instance->CCR1 = 0;
	}
	else if(motor_config_params[motor_instance].PWM_TIM_CH == TIM_CHANNEL_2)
	{
		motor_config_params[motor_instance].TIM_Instance->CCR2 = 0;
	}
	else if(motor_config_params[motor_instance].PWM_TIM_CH == TIM_CHANNEL_3)
	{
		motor_config_params[motor_instance].TIM_Instance->CCR3 = 0;
	}
	else
	{
		motor_config_params[motor_instance].TIM_Instance->CCR4 = 0;
	}

}

