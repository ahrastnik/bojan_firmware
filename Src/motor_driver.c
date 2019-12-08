/*
 * motor_driver.c
 *
 *  Created on: 14. okt. 2019
 *      Author: adamh
 */

#include "motor_driver.h"

#include <math.h>

#include "main.h"

// Macros
#define ENCODER_CPR	(48.0f) // Counts per revolution
#define GEAR_RATIO (20.4f)	// 20.4:1
#define THREAD_PITCH (2.4f) // [mm]
#define MOTOR_STEP (THREAD_PITCH / (ENCODER_CPR * GEAR_RATIO))
#define MOTOR_MAX_RPS (370.0f / 60.0f) // Max rotations per seconds at no load (12V)


// Private function declarations
/**
 * Update the moving axis
 *
 * @param axis	Axis to update
 * @param time	Current Systick in milliseconds
 */
static void _move_axis(axis_t axis, uint32_t time);
/**
 * Stop an axis
 *
 * @param axis	Axis to stop
 */
static void _stop_axis(axis_t axis);
/**
 * Get the current absolute position of the axis
 *
 * @param axis			Axis for which to update the absolute position
 * @param encoder_count	Current axis encoder timer count
 */
static void _update_axis_absolute_position(axis_t axis, int16_t encoder_count);

// Motor and encoder timers
static TIM_HandleTypeDef *_motor_a_timer;
static TIM_HandleTypeDef *_motor_a_encoder_timer;
static TIM_HandleTypeDef *_motor_b_timer;
static TIM_HandleTypeDef *_motor_b_encoder_timer;

// Absolute tool position
static volatile state_t _state = {
		.position = {
				.x = 0.0,
				.y = 0.0,
		},
		.move_start_time_x = 0,
		.move_start_x = 0.0,
		.move_end_x = 0.0,
		.move_start_time_y = 0,
		.move_start_y = 0.0,
		.move_end_y = 0.0,
		.feedrate = 0.0,
		.z = 0
};

// Encoders
static volatile position_encoder_t _motor_a_encoder = {
		.old_encoder_count = 0,
		.encoder_count_absolute = 0
};
static volatile position_encoder_t _motor_b_encoder = {
		.old_encoder_count = 0,
		.encoder_count_absolute = 0
};


void motor_driver_init(
		TIM_HandleTypeDef *motor_a_timer,
		TIM_HandleTypeDef *motor_a_encoder_timer,
		TIM_HandleTypeDef *motor_b_timer,
		TIM_HandleTypeDef *motor_b_encoder_timer) {

	// Store timer pointers for internal use
	_motor_a_timer = motor_a_timer;
	_motor_a_encoder_timer = motor_a_encoder_timer;
	_motor_b_timer = motor_b_timer;
	_motor_b_encoder_timer = motor_b_encoder_timer;

	// Motor A
	axis_x_enable();
	HAL_TIM_PWM_Start(motor_a_timer, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(motor_a_timer, TIM_CHANNEL_2);

	HAL_TIM_Encoder_Start(motor_a_encoder_timer, TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start(motor_a_encoder_timer, TIM_CHANNEL_2);

	// Motor B
	axis_y_enable();
	HAL_TIM_PWM_Start(motor_b_timer, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(motor_b_timer, TIM_CHANNEL_2);

	HAL_TIM_Encoder_Start(motor_b_encoder_timer, TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start(motor_b_encoder_timer, TIM_CHANNEL_2);
}

void axis_x_enable(void) {
	HAL_GPIO_WritePin(EN_A_GPIO_Port, EN_A_Pin, GPIO_PIN_SET);
}

void axis_x_disable(void) {
	stop_x();
	HAL_GPIO_WritePin(EN_A_GPIO_Port, EN_A_Pin, GPIO_PIN_RESET);
}

void axis_y_enable(void) {
	HAL_GPIO_WritePin(EN_B_GPIO_Port, EN_B_Pin, GPIO_PIN_SET);
}

void axis_y_disable(void) {
	stop_y();
	HAL_GPIO_WritePin(EN_B_GPIO_Port, EN_B_Pin, GPIO_PIN_RESET);
}


static void _move_axis(axis_t axis, uint32_t time) {
	// Get the requested axis motor timer
	TIM_HandleTypeDef *motor_timer;
	float start_position;
	float end_position;
	float feedrate;
	float move_time;

	switch (axis) {
		case AXIS_X:
			motor_timer = _motor_a_timer;
			move_time = ((float)(time - _state.move_start_time_x) / 1000);
			start_position = _state.move_start_x;
			end_position = _state.move_end_x;
			feedrate = _state.feedrate;
			break;

		case AXIS_Y:
			motor_timer = _motor_b_timer;
			move_time = ((float)(time - _state.move_start_time_y) / 1000);
			start_position = _state.move_start_y;
			end_position = _state.move_end_y;
			feedrate = _state.feedrate;
			break;

		default:
			return;
	}

	// Calculate the axis position difference after move
	float relative_position = end_position - start_position;
	if (relative_position == 0) {
		_stop_axis(axis);
		return;
	}
	float absolute_relative_position = fabsf(relative_position);
	float time_to_arrival = absolute_relative_position / feedrate;
	if (move_time > time_to_arrival) {
		_stop_axis(axis);
		return;
	}

	// Calculate the new velocity increment
	float velocity_increment = (-6 / pow(time_to_arrival, 3))*absolute_relative_position*pow(move_time, 2) + (6 / pow(time_to_arrival, 2))*absolute_relative_position*move_time;
	velocity_increment = velocity_increment < 0 ? 0 : velocity_increment;

	// Calculate the PWM duty cycle for this increment
	float rotations_per_second = velocity_increment / THREAD_PITCH;
	// Convert to duty cycle in %
	int32_t duty_cycle = (rotations_per_second / MOTOR_MAX_RPS) * 100;
	// Clamp duty cycle to 100% max and 0% minimum
	if (duty_cycle > 100) {
		duty_cycle = 100;
	} else if (duty_cycle < 0) {
		duty_cycle = 0;
	}

	// Set motor PWM duty cycle
	if (relative_position > 0) {
		motor_timer->Instance->CCR1 = duty_cycle;
		motor_timer->Instance->CCR2 = 0;
	} else if (relative_position < 0) {
		motor_timer->Instance->CCR1 = 0;
		motor_timer->Instance->CCR2 = duty_cycle;
	} else {
		// Stop motor
		_stop_axis(axis);
	}
}

void move_x(float position, float feedrate) {
	_state.move_start_time_x = HAL_GetTick();
	_state.move_end_x = position;
	_state.move_start_x = _state.position.x;
	_state.feedrate = feedrate;
}

void move_y(float position, float feedrate) {
	_state.move_start_time_y = HAL_GetTick();
	_state.move_end_y = position;
	_state.move_start_y = _state.position.y;
	_state.feedrate = feedrate;
}

void move(vector_2d_t position, float feedrate) {
	move_x(position.x, feedrate);
	move_y(position.y, feedrate);
}

static void _stop_axis(axis_t axis) {
	switch (axis) {
		case AXIS_X:
			_motor_a_timer->Instance->CCR1 = 0;
			_motor_a_timer->Instance->CCR2 = 0;
			_state.move_start_time_x = 0;
			_state.move_start_x = 0;
			_state.move_end_x = 0;
			return;

		case AXIS_Y:
			_motor_b_timer->Instance->CCR1 = 0;
			_motor_b_timer->Instance->CCR2 = 0;
			_state.move_start_time_y = 0;
			_state.move_start_y = 0;
			_state.move_end_y = 0;
			return;

		default:
			return;
	}
}

void stop_x(void) {
	_stop_axis(AXIS_X);
}

void stop_y(void) {
	_stop_axis(AXIS_Y);
}

void stop(void) {
	_stop_axis(AXIS_X);
	_stop_axis(AXIS_Y);
}

void home_x(void) {
	_motor_a_timer->Instance->CCR1 = 50;
	while(HAL_GPIO_ReadPin(HOME_X_GPIO_Port, HOME_X_Pin));
	_motor_a_timer->Instance->CCR1 = 0;
}

void home_y(void) {
	_motor_b_timer->Instance->CCR1 = 50;
	while(HAL_GPIO_ReadPin(HOME_Y_GPIO_Port, HOME_Y_Pin));
	_motor_b_timer->Instance->CCR1 = 0;
}

void home(void) {
	home_x();
	home_y();
}

static void _update_axis_absolute_position(axis_t axis, int16_t encoder_count) {
	volatile position_encoder_t *encoder;

	// Choose the encoder based on the requested axis
	switch (axis) {
		case AXIS_X:
			encoder = &_motor_a_encoder;
			break;

		case AXIS_Y:
			encoder = &_motor_b_encoder;
			break;

		default:
			return;
	}

	// Calculate the number of counts the encoder has counted since the last update
	int16_t encoder_relative = encoder_count - encoder->old_encoder_count;
	encoder->encoder_count_absolute += encoder_relative;

	// Calculate the absolute position of the axis
	float absolute_position = (float)encoder->encoder_count_absolute * MOTOR_STEP;
	switch (axis) {
		case AXIS_X:
			_state.position.x = absolute_position;
			break;

		case AXIS_Y:
			_state.position.y = absolute_position;
			break;

		default:
			break;
	}
	// Reset the relative counter
	encoder->old_encoder_count = encoder_count;
}

void motor_driver_update(void) {
	// Get the absolute position
	_update_axis_absolute_position(AXIS_X, TIM1->CNT);
	_update_axis_absolute_position(AXIS_Y, TIM4->CNT);
	// Update the incremental move controller
	uint32_t time = HAL_GetTick();
	_move_axis(AXIS_X, time);
	_move_axis(AXIS_Y, time);
}

state_t get_state(void) {
	return _state;
}
