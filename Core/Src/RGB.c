/*
 * RGB.c
 *
 *  Created on: Feb 13, 2024
 *      Author: Edwin Mwiti
 */

#include "RGB.h"

/* set RGB LED parameters */
void RGB_set_parameters(RGB_instance* rgb_instance, TIM_HandleTypeDef* timer, uint32_t last_time) {
	rgb_instance->timer = timer;
	rgb_instance->fade_interval = FADE_INTERVAL;
	rgb_instance->last_time = last_time;
}

/* start the RGB instance */
void RGB_init(RGB_instance* rgb_instance) {

	// start the channels for RGB
	HAL_TIM_PWM_Start (rgb_instance->timer, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start (rgb_instance->timer, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start (rgb_instance->timer, TIM_CHANNEL_3);

	// set initial colors
	rgb_instance->timer->Instance->CCR1 = 0;
	rgb_instance->timer->Instance->CCR2 = 255;
	rgb_instance->timer->Instance->CCR3 = 255;

}

/* set RGB color */
void RGB_set_color(RGB_instance* rgb_instance, RGB_color* color) {

	// set color
	rgb_instance->red = color->red_value;
	rgb_instance->green = color->green_value;
	rgb_instance->blue = color->blue_value;

	// set the CCR values for the given RGB LED instance
	rgb_instance->timer->Instance->CCR1 = rgb_instance->red;
	rgb_instance->timer->Instance->CCR2 = rgb_instance->green;
	rgb_instance->timer->Instance->CCR3 = rgb_instance->blue;

}

/**
 * RGB LED color states
 *
 */

/* slow fade in out white */
void RGB_slow_fade_in_out(RGB_instance* rgb_instance) {
	for(int i =0; i < 255; i++) {
		rgb_instance->red = i;
		rgb_instance->green = i;
		rgb_instance->blue = i;
	}
}

/* fast fade in out */
void fast_fade_in_out(RGB_instance* rgb_instance);

/* pulse red */
void pulse(RGB_instance* rgb_instance, RGB_color_code color_code , uint32_t last_time){
	RGB_color color;
	RGB_color* p_color = &color;

	if(color_code == RED) {
		// red color

		// increase brightness
		for(int i =0; i < 255; i++) {
			p_color->red_value = i;
			p_color->green_value = 0;
			p_color->blue_value = 0;

			// non-blocking set color
			if( (HAL_GetTick() - rgb_instance->last_time) > rgb_instance->fade_interval ) {
				RGB_set_color(rgb_instance, p_color);
				rgb_instance->last_time = HAL_GetTick();
			}

		}

		// decrease brightness
		for(int i = 255; i > 0; i--) {
			p_color->red_value = i;
			p_color->green_value = 0;
			p_color->blue_value = 0;

			// non-blocking set color
			if( (HAL_GetTick() - rgb_instance->last_time) > rgb_instance->fade_interval ) {
				RGB_set_color(rgb_instance, p_color);
				rgb_instance->last_time = HAL_GetTick();
			}
		}

	} else if (color_code == GREEN ) {
		// green color

		// increase brightness
		for(int i =0; i < 255; i++) {
			p_color->red_value = 0;
			p_color->green_value = i;
			p_color->blue_value = 0;

			// non-blocking set color
			if( (HAL_GetTick() - rgb_instance->last_time) > rgb_instance->fade_interval ) {
				RGB_set_color(rgb_instance, p_color);
				rgb_instance->last_time = HAL_GetTick();
			}

		}

		// decrease brightness
		for(int i = 255; i > 0; i--) {
			p_color->red_value = 0;
			p_color->green_value = i;
			p_color->blue_value = 0;

			// non-blocking set color
			if( (HAL_GetTick() - rgb_instance->last_time) > rgb_instance->fade_interval ) {
				RGB_set_color(rgb_instance, p_color);
				rgb_instance->last_time = HAL_GetTick();
			}
		}
	} else if(color_code == BLUE) {
		// blue color

		// increase brightness
		for(int i =0; i < 255; i++) {
			p_color->red_value = 0;
			p_color->green_value = 0;
			p_color->blue_value = i;

			// non-blocking set color
			if( (HAL_GetTick() - rgb_instance->last_time) > rgb_instance->fade_interval ) {
				RGB_set_color(rgb_instance, p_color);
				rgb_instance->last_time = HAL_GetTick();
			}

		}

		// decrease brightness
		for(int i = 255; i > 0; i--) {
			p_color->red_value = 0;
			p_color->green_value = 0;
			p_color->blue_value = i;

			// non-blocking set color
			if( (HAL_GetTick() - rgb_instance->last_time) > rgb_instance->fade_interval ) {
				RGB_set_color(rgb_instance, p_color);
				rgb_instance->last_time = HAL_GetTick();
			}
		}
	} else if(color_code == WHITE) {
		// white color
		// increase brightness
		for(int i =0; i < 255; i++) {
			p_color->red_value = i;
			p_color->green_value = i;
			p_color->blue_value = i;

			// non-blocking set color
			if( (HAL_GetTick() - rgb_instance->last_time) > rgb_instance->fade_interval ) {
				RGB_set_color(rgb_instance, p_color);
				rgb_instance->last_time = HAL_GetTick();
			}

		}

		// decrease brightness
		for(int i = 255; i > 0; i--) {
			p_color->red_value = i;
			p_color->green_value = i;
			p_color->blue_value = i;

			// non-blocking set color
			if( (HAL_GetTick() - rgb_instance->last_time) > rgb_instance->fade_interval ) {
				RGB_set_color(rgb_instance, p_color);
				rgb_instance->last_time = HAL_GetTick();
			}
		}
	}

}

