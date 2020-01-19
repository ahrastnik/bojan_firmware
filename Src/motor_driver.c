/*
 * motor_driver.c
 *
 *  Created on: 14. okt. 2019
 *      Author: adamh
 */

#include "motor_driver.h"

#include <math.h>
#include <stdbool.h>
#include <stdio.h>

#include "main.h"

// Machine parameters
#define WORKSPACE_X			(100.0f) // 100 mm on X axis
#define WORKSPACE_Y			(100.0f) // 100 mm on Y axis
#define ENCODER_CPR			(48.0f) // Counts per revolution
#define GEAR_RATIO 			(99.0f)	// 99:1
#define THREAD_PITCH 		(25.4f) // [mm]
#define MOTOR_STEP 			(THREAD_PITCH / (ENCODER_CPR * GEAR_RATIO))
#define MOTOR_MAX_RPS 		(220.0f / 60.0f) // Max rotations per seconds at no load (12V)
#define MOTOR_MAX_FEEDRATE 	(MOTOR_MAX_RPS * THREAD_PITCH) // [mm/s]
#define DEFAULT_FEEDRATE 	(MOTOR_MAX_FEEDRATE / 2) // [mm/s]
// Servo (brush)
#define SERVO_FREQUENCY 	(50.0f) // [Hz]
#define SERVO_DUTY_RANGE 	1000 // Duty cycle range
#define SERVO_TIME_RANGE	(1000.0f / SERVO_FREQUENCY) // [ms]
#define SERVO_TO_DUTY(X)	((X / SERVO_TIME_RANGE) * SERVO_DUTY_RANGE)
#define SERVO_RIGHT 		SERVO_TO_DUTY(1.0f) // [ms]
#define SERVO_CENTER		SERVO_TO_DUTY(1.5f) // [ms]
#define SERVO_LEFT 			SERVO_TO_DUTY(2.0f) // [ms]
// PID regulator
#define KP 						(1.0f)
#define KI 						(0.001f)
#define KD 						(0.005f)
#define ALLOWED_ERROR_MARGIN 	(0.05f) // [mm]
#define ERROR_MARGIN_CHECKS		8


// Private function declarations
/**
 * Convert feedrate to PWM duty cycle
 *
 * @param 	feedrate	Feedrate in [mm/s]
 *
 * @returns PWM duty cycle [0 - 100]
 */
static uint32_t _feedrate_to_duty_cycle(float feedrate);
/**
 * Jog the selected axis
 *
 * @param feedrate	Feedrate at which to jog the axis
 */
static void _jog_axis(axis_t axis, float feedrate);
/**
 * Update the moving axis
 *
 * @param axis	Axis to update
 */
static void _move_axis(axis_t axis);
/**
 * Stop an axis
 *
 * @param axis	Axis to stop
 * @param clear	Clear the axis data
 */
static void _stop_axis(axis_t axis, bool clear);
/**
 * Clamp axis to the defined workspace
 *
 * @param axis	Axis to clamp
 */
static void _clamp_axis(axis_t axis);
/**
 * Home axis
 *
 * @param axis	Axis to home
 */
static void _home_axis(axis_t axis);
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
static TIM_HandleTypeDef *_servo_motor_timer;

// Absolute tool position
static volatile state_t _state = {
		.position = {
				.x = 0.0,
				.y = 0.0,
		},
		.z = false,
		.axis_x = {
				.last_sample_time = 0,
				.move_start_time = 0,
				.last_position = 0.0,
				.move_start = 0.0,
				.move_end = 0.0,
				.last_error = 0.0,
				.cumulative_error = 0.0,
				.margin_check_counter = 0,
				.moving = false,
				.homing = false
		},
		.axis_y = {
				.last_sample_time = 0,
				.move_start_time = 0,
				.last_position = 0.0,
				.move_start = 0.0,
				.move_end = 0.0,
				.last_error = 0.0,
				.cumulative_error = 0.0,
				.margin_check_counter = 0,
				.moving = false,
				.homing = false
		},
		.feedrate = DEFAULT_FEEDRATE,
		.position_mode = POSITIONING_ABSOLUTE
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
		TIM_HandleTypeDef *motor_b_encoder_timer,
		TIM_HandleTypeDef *servo_motor_timer) {

	// Store timer pointers for internal use
	_motor_a_timer = motor_a_timer;
	_motor_a_encoder_timer = motor_a_encoder_timer;
	_motor_b_timer = motor_b_timer;
	_motor_b_encoder_timer = motor_b_encoder_timer;
	_servo_motor_timer = servo_motor_timer;

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

	// Servo motor
	HAL_TIM_PWM_Start(servo_motor_timer, TIM_CHANNEL_1);
	_servo_motor_timer->Instance->CCR1 = SERVO_CENTER;
}

void axis_x_enable(void) {
	HAL_GPIO_WritePin(EN_A_GPIO_Port, EN_A_Pin, GPIO_PIN_SET);
}

void axis_x_disable(void) {
	stop_x(false);
	HAL_GPIO_WritePin(EN_A_GPIO_Port, EN_A_Pin, GPIO_PIN_RESET);
}

void axis_y_enable(void) {
	HAL_GPIO_WritePin(EN_B_GPIO_Port, EN_B_Pin, GPIO_PIN_SET);
}

void axis_y_disable(void) {
	stop_y(false);
	HAL_GPIO_WritePin(EN_B_GPIO_Port, EN_B_Pin, GPIO_PIN_RESET);
}

void axis_x_zero(void) {
	_state.position.x = 0;
}

void axis_y_zero(void) {
	_state.position.y = 0;
}

static void _jog_axis(axis_t axis, float feedrate) {
	TIM_HandleTypeDef *motor_timer;
	volatile bool *moving;

	switch (axis) {
		case AXIS_X:
			motor_timer = _motor_a_timer;
			moving = &_state.axis_x.moving;
			break;

		case AXIS_Y:
			motor_timer = _motor_b_timer;
			moving = &_state.axis_y.moving;
			break;

		default:
			return;
	}

	*moving = true;

	if (feedrate > 0.0) {
		motor_timer->Instance->CCR1 = _feedrate_to_duty_cycle(fabsf(feedrate));
		motor_timer->Instance->CCR2 = 0;

	} else if (feedrate < 0.0) {
		motor_timer->Instance->CCR1 = 0;
		motor_timer->Instance->CCR2 = _feedrate_to_duty_cycle(fabsf(feedrate));

	} else {
		_stop_axis(axis, false);
	}
}

void jog_x(float feedrate) {
	_jog_axis(AXIS_X, feedrate);
	printf(COMMAND_FINISHED_REPLY);
}

void jog_y(float feedrate) {
	_jog_axis(AXIS_Y, feedrate);
	printf(COMMAND_FINISHED_REPLY);
}

void jog(vector_2d_t feedrate) {
	_jog_axis(AXIS_X, feedrate.x);
	_jog_axis(AXIS_Y, feedrate.y);
	printf(COMMAND_FINISHED_REPLY);
}

static uint32_t _feedrate_to_duty_cycle(float feedrate) {
	// Clamp feedrate
	feedrate = feedrate > MOTOR_MAX_FEEDRATE ? MOTOR_MAX_FEEDRATE : feedrate;
	// Calculate the PWM duty cycle for this increment
	float rotations_per_second = feedrate / THREAD_PITCH;
	// Convert to duty cycle in %
	int32_t duty_cycle = (rotations_per_second / MOTOR_MAX_RPS) * 100;
	// Clamp duty cycle to 100% max and 0% minimum
	if (duty_cycle > 100) {
		duty_cycle = 100;
	} else if (duty_cycle < 0) {
		duty_cycle = 0;
	}
	return (uint32_t)duty_cycle;
}

static void _move_axis(axis_t axis) {
	// Get the requested axis motor timer
	uint32_t time = HAL_GetTick();
	TIM_HandleTypeDef *motor_timer;
	float current_position;
	float start_position;
	float end_position;
	float move_time;
	float dt;
	volatile float *cumulative_error;
	volatile float *last_error;
	volatile uint8_t *margin_check_counter;
	volatile bool *moving;

	float feedrate = _state.feedrate;

	switch (axis) {
		case AXIS_X:
			motor_timer = _motor_a_timer;
			current_position = _state.position.x;
			move_time = ((float)(time - _state.axis_x.move_start_time) / 1000);
			dt = ((float)(time - _state.axis_x.last_sample_time) / 1000);
			_state.axis_x.last_sample_time = time;
			_state.axis_x.last_position = _state.position.x;
			start_position = _state.axis_x.move_start;
			end_position = _state.axis_x.move_end;
			last_error = &_state.axis_x.last_error;
			cumulative_error = &_state.axis_x.cumulative_error;
			margin_check_counter = &_state.axis_x.margin_check_counter;
			moving = &_state.axis_x.moving;
			break;

		case AXIS_Y:
			motor_timer = _motor_b_timer;
			current_position = _state.position.y;
			move_time = ((float)(time - _state.axis_y.move_start_time) / 1000);
			dt = ((float)(time - _state.axis_y.last_sample_time) / 1000);
			_state.axis_y.last_sample_time = time;
			_state.axis_y.last_position = _state.position.y;
			start_position = _state.axis_y.move_start;
			end_position = _state.axis_y.move_end;
			last_error = &_state.axis_y.last_error;
			cumulative_error = &_state.axis_y.cumulative_error;
			margin_check_counter = &_state.axis_y.margin_check_counter;
			moving = &_state.axis_y.moving;
			break;

		default:
			return;
	}

	// Check if the axis is active
	if (!*moving) {
		return;
	}

	// Calculate the position error
	float error = end_position - current_position;

	// Check if the position was already reached and stable in the last two cycles
	// TODO: If the error is the same in the last few (e.g. 4) cycles, finish the move immediately
	if (error <= ALLOWED_ERROR_MARGIN &&
			error >= -ALLOWED_ERROR_MARGIN) {

		if (*margin_check_counter < (ERROR_MARGIN_CHECKS - 1)) {
			// Buffer the error
			(*margin_check_counter)++;
		} else {
			// Notify about the finished move
			if ((*moving && !_state.axis_x.moving) ||
					(*moving && !_state.axis_y.moving)) {
				printf(COMMAND_FINISHED_REPLY);
			}
			*moving = false;
			// Reset the error margin counter
			*margin_check_counter = 0;
		}
		_stop_axis(axis, false);
		return;

	} else {
		// Reset the error margin counter
		*margin_check_counter = 0;
	}

	// Calculate the axis position difference after move
	float relative_position = end_position - start_position;

	// Apply the S-curve, if the relative position is valid
	if (relative_position != 0) {
		float absolute_relative_position = fabsf(relative_position);
		// Clamp feedrate
		feedrate = feedrate > MOTOR_MAX_FEEDRATE ? MOTOR_MAX_FEEDRATE : feedrate;
		// Calculate time to arrival
		float time_to_arrival = (3*absolute_relative_position) / (2*feedrate);

		// Calculate the S-curve
		float position = (-2 / pow(time_to_arrival, 3))*relative_position*pow(move_time, 3) + (3 / pow(time_to_arrival, 2))*relative_position*pow(move_time, 2)+start_position;
		// Clamp S-curve by time
		position = move_time > time_to_arrival ? end_position : position;
		error = position - current_position;
	}

	// Calculate other PID regulator parameters
	*cumulative_error += error * dt;
	float rate_error = (error - *last_error) / dt;
	*last_error = error;
	// Calculate the PID output
	float pid_out = KP * error + KI * (*cumulative_error) + KD * rate_error;
	/*if (axis == AXIS_X) {
		printf("Axis: %d\nCurrent position: %f\nEnd position: %f\nPID output: %f\n", axis, current_position, end_position, pid_out);
	}*/

	// Calculate the PWM duty cycle for this increment
	uint32_t duty_cycle = _feedrate_to_duty_cycle(fabsf(pid_out/dt));

	// Set axis motor PWM duty cycle
	if (error > 0) {
		motor_timer->Instance->CCR1 = duty_cycle;
		motor_timer->Instance->CCR2 = 0;

	} else if (error < 0) {
		motor_timer->Instance->CCR1 = 0;
		motor_timer->Instance->CCR2 = duty_cycle;

	} else {
		// Stop motor
		_stop_axis(axis, false);
	}
}

void move_x(float position, float feedrate) {
	_state.axis_x.move_start_time = HAL_GetTick();
	_state.axis_x.move_start = _state.position.x;
	_state.axis_x.moving = true;
	_state.feedrate = feedrate;

	// Set the new move position
	if (_state.position_mode == POSITIONING_ABSOLUTE) {
		_state.axis_x.move_end = position;
	} else {
		_state.axis_x.move_end += position;
	}

	// Clamp the move position to the available workspace
#ifndef DEBUG
	if (_state.axis_x.move_end > WORKSPACE_X) {
		_state.axis_x.move_end = WORKSPACE_X;
	} else if (_state.axis_x.move_end < 0.0) {
		_state.axis_x.move_end = 0.0;
	}
#endif
}

void move_y(float position, float feedrate) {
	_state.axis_y.move_start_time = HAL_GetTick();
	_state.axis_y.move_start = _state.position.y;
	_state.axis_y.moving = true;
	_state.feedrate = feedrate;

	if (_state.position_mode == POSITIONING_ABSOLUTE) {
		_state.axis_y.move_end = position;
	} else {
		_state.axis_y.move_end += position;
	}

#ifndef DEBUG
	if (_state.axis_y.move_end > WORKSPACE_Y) {
		_state.axis_y.move_end = WORKSPACE_Y;
	} else if (_state.axis_y.move_end < 0.0) {
		_state.axis_y.move_end = 0.0;
	}
#endif
}

void move(vector_2d_t position, float feedrate) {
	move_x(position.x, feedrate);
	move_y(position.y, feedrate);
}

void brush_drop(void) {
	if (_state.z) return;

	_servo_motor_timer->Instance->CCR1 = SERVO_LEFT;
	_state.z = true;
}

void brush_raise(void) {
	if (!_state.z) return;

	_servo_motor_timer->Instance->CCR1 = SERVO_RIGHT;
	_state.z = false;
}

static void _clamp_axis(axis_t axis) {
	float position;
	float max_position;
	float min_position;

	switch (axis) {
		case AXIS_X:
			position = _state.position.x;
			max_position = WORKSPACE_X;
			min_position = 0.0;
			break;

		case AXIS_Y:
			position = _state.position.y;
			max_position = WORKSPACE_Y;
			min_position = 0.0;
			break;

		default:
			return;
	}

	// Stop the axis, if it has breached the allowed workspace
	if (position > max_position ||
			position < min_position) {
		_stop_axis(axis, true);
	}
}

static void _stop_axis(axis_t axis, bool clear) {
	// Reset axis specific variables
	switch (axis) {
		case AXIS_X:
			_motor_a_timer->Instance->CCR1 = 0;
			_motor_a_timer->Instance->CCR2 = 0;

			if (clear) {
				_state.axis_x.last_sample_time = 0;
				_state.axis_x.move_start_time = 0;
				_state.axis_x.move_start = 0;
				_state.axis_x.move_end = 0;
				_state.axis_x.last_error = 0.0;
				_state.axis_x.last_position = 0.0;
				_state.axis_x.cumulative_error = 0.0;
				_state.axis_x.moving = false;
				_state.axis_x.homing = false;
			}
			return;

		case AXIS_Y:
			_motor_b_timer->Instance->CCR1 = 0;
			_motor_b_timer->Instance->CCR2 = 0;

			if (clear) {
				_state.axis_y.last_sample_time = 0;
				_state.axis_y.move_start_time = 0;
				_state.axis_y.move_start = 0;
				_state.axis_y.move_end = 0;
				_state.axis_y.last_error = 0.0;
				_state.axis_y.last_position = 0.0;
				_state.axis_y.cumulative_error = 0.0;
				_state.axis_y.moving = false;
				_state.axis_y.homing = false;
			}
			return;

		default:
			return;
	}
}

void stop_x(bool reply) {
	_stop_axis(AXIS_X, true);
	if (reply) {
		printf(COMMAND_FINISHED_REPLY);
	}
}

void stop_y(bool reply) {
	_stop_axis(AXIS_Y, true);
	if (reply) {
		printf(COMMAND_FINISHED_REPLY);
	}
}

void stop(void) {
	_stop_axis(AXIS_X, true);
	_stop_axis(AXIS_Y, true);
	printf(COMMAND_FINISHED_REPLY);
}

static void _home_axis(axis_t axis) {
	GPIO_TypeDef *port;
	uint16_t pin;
	volatile bool *homing;

	switch (axis) {
		case AXIS_X:
			homing = &_state.axis_x.homing;
			port = HOME_X_GPIO_Port;
			pin = HOME_X_Pin;
			break;

		case AXIS_Y:
			homing = &_state.axis_y.homing;
			port = HOME_Y_GPIO_Port;
			pin = HOME_Y_Pin;
			break;

		default:
			return;
	}

	if (!HAL_GPIO_ReadPin(port, pin)) {
		return;
	}
	*homing = true;
	_jog_axis(axis, -DEFAULT_FEEDRATE);
}

void home_x(void) {
	_home_axis(AXIS_X);
}

void home_y(void) {
	_home_axis(AXIS_Y);
}

void home(void) {
	_home_axis(AXIS_X);
	_home_axis(AXIS_Y);
}

void positioning_absolute() {
	_state.position_mode = POSITIONING_ABSOLUTE;
	printf(COMMAND_FINISHED_REPLY);
}

void positioning_relative() {
	_state.position_mode = POSITIONING_RELATIVE;
	printf(COMMAND_FINISHED_REPLY);
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
#ifndef DEBUG
	// Prevent axis from moving if it breached the allowed workspace
	_clamp_axis(AXIS_X);
	_clamp_axis(AXIS_Y);
#endif
	// Get the absolute position
	_update_axis_absolute_position(AXIS_X, TIM1->CNT);
	_update_axis_absolute_position(AXIS_Y, TIM4->CNT);
	// Update the incremental move controller
	_move_axis(AXIS_X);
	_move_axis(AXIS_Y);
}

state_t get_state(void) {
	return _state;
}
