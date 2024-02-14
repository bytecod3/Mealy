/*
 * RGB.h
 *
 *  Created on: Feb 13, 2024
 *      Author: Edwin Mwiti
 */

#ifndef INC_RGB_H_
#define INC_RGB_H_

#include <stm32f1xx_hal.h>

typedef struct {
	TIM_HandleTypeDef* timer;	// timer being used for RGB LED
	uint32_t last_time; // to implement non-blocking RGB
	uint8_t fade_interval;		// interval timer to fade LEDs. set small number for fast fade
	uint8_t red;				// red color value
	uint8_t green;				// green color value
	uint8_t blue;				// blue color value

} RGB_instance;

typedef struct {
	uint8_t red_value;			// red color value
	uint8_t green_value;		// green color value
	uint8_t blue_value;			// blue color value
} RGB_color;

typedef enum {
	RED = 0 ,
	GREEN,
	BLUE,
	WHITE
}  RGB_color_code;

#define FADE_INTERVAL 1000 // interval timer to fade LEDs. set small number for fast fade

/* set RGB LED parameters */
void RGB_set_parameters(RGB_instance* rgb_instance, TIM_HandleTypeDef* timer, uint32_t last_time);

/* start the RGB instance */
void RGB_init(RGB_instance* rgb_instance);

/* set rgb color */
void RGB_set_color(RGB_instance* rgb_instance, RGB_color* color);

/**
 * RGB LED color states
 *
 */

/* slow fade in out */
void RGB_slow_fade_in_out(RGB_instance* rgb_instance);

/* fast fade in out */
void fast_fade_in_out(RGB_instance* rgb_instance);

/* pulse a single color */
void pulse(RGB_instance* rgb_instance, RGB_color_code color,  uint32_t last_time);


#endif /* INC_RGB_H_ */
