/*
 * led_driver.c
 *
 *  Created on: 16. jan. 2020
 *      Author: adamh
 */

#include "led_driver.h"

static TIM_HandleTypeDef *_led_driver_timer;
static TIM_HandleTypeDef *_led_handler_timer;

static volatile led_driver_state_t _state = {
	.enabled = false,
	.mode = DISPLAY_MODE_RAINBOW,
	.color = 0x3FFFFFFF
};

/**
 * Set color of a single channel
 */
static void _color_channel_set(color_channels_t channel, color_channel_t color);

void led_driver_init(TIM_HandleTypeDef *led_driver_timer,
					 TIM_HandleTypeDef *led_handler_timer) {
	// Store timer reference
	_led_driver_timer = led_driver_timer;
	_led_handler_timer = led_handler_timer;

	// Start all PWM channels
	HAL_TIM_PWM_Start(led_driver_timer, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(led_driver_timer, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(led_driver_timer, TIM_CHANNEL_3);
}

void led_driver_enable(void) {
	_state.enabled = true;

	// Turn off the LEDs
	color_set(_state.color);
	// Start the handler timer
	HAL_TIM_Base_Start_IT(_led_handler_timer);
}

void led_driver_disable(void) {
	// Turn off the LEDs
	color_set(0x00000000);
	// Stop the handler timer
	HAL_TIM_Base_Stop_IT(_led_handler_timer);

	_state.enabled = false;
}

static void _color_channel_set(color_channels_t channel, color_channel_t color) {

	switch (channel) {
		case COLOR_CHANNEL_R:
			_led_driver_timer->Instance->CCR1 = color;
			break;

		case COLOR_CHANNEL_G:
			_led_driver_timer->Instance->CCR2 = color;
			break;

		case COLOR_CHANNEL_B:
			_led_driver_timer->Instance->CCR3 = color;
			break;

		default:
			return;
	}
}

void color_set(color_t color) {
	// R - channel
	_color_channel_set(COLOR_CHANNEL_R, (color >> 20) & 0x3FF);
	// G - channel
	_color_channel_set(COLOR_CHANNEL_G, (color >> 10) & 0x3FF);
	// B - channel
	_color_channel_set(COLOR_CHANNEL_B, color & 0x3FF);
}

color_t color_get(void) {
	return _state.color;
}

void mode_set(display_modes_t mode) {
	_state.mode = mode;
}

display_modes_t mode_get(void) {
	return _state.mode;
}

void led_driver_update(void) {
	if (!_state.enabled) {
		return;
	}
	switch (_state.mode) {
		case DISPLAY_MODE_RAINBOW:
			color_set(_state.color++);
			break;

		default:
			return;
	}
}
