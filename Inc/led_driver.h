/*
 * led_driver.h
 *
 *  Created on: 16. jan. 2020
 *      Author: adamh
 */

#ifndef LED_DRIVER_H_
#define LED_DRIVER_H_

#include <stdint.h>
#include <stdbool.h>

#include "main.h"

typedef enum DisplayModes {
	DISPLAY_MODE_RAINBOW,
	DISPLAY_MODE_NUM
} display_modes_t;

typedef enum ColorChannels {
	COLOR_CHANNEL_R,
	COLOR_CHANNEL_G,
	COLOR_CHANNEL_B
} color_channels_t;

typedef uint32_t color_t; // 30 bits -> RGB
typedef uint16_t color_channel_t; // 10 bits per channel

typedef struct LedDriverState {
	bool enabled;
	display_modes_t mode;
	color_t color;
} led_driver_state_t;

/**
 * Initialize the LED driver
 *
 * @param led_driver_timer	LED driver timer (PWM) reference
 * @param led_handler_timer	LED handler timer reference
 */
void led_driver_init(TIM_HandleTypeDef *led_driver_timer,
					 TIM_HandleTypeDef *led_handler_timer);

/**
 * Enable the LED driver
 * This activates the LED handler
 */
void led_driver_enable(void);

/**
 * Disable the LED driver
 * This deactivates the LED handler
 */
void led_driver_disable(void);

/**
 * Set color of the LED driver
 *
 * @param color	The color to set
 */
void color_set(color_t color);

/**
 * Get the current color of the LED driver
 */
color_t color_get(void);

/**
 * Set a new display mode
 *
 * @param mode	The display mode to set
 */
void mode_set(display_modes_t mode);

/**
 * Get the current display mode
 */
display_modes_t mode_get(void);

/**
 * Update the led handler
 */
void led_driver_update(void);

#endif /* LED_DRIVER_H_ */
