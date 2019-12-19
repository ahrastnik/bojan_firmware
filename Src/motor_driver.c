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
#define ENCODER_CPR			(48.0f) // Counts per revolution
#define GEAR_RATIO 			(20.4f)	// 20.4:1
#define THREAD_PITCH 		(2.4f) // [mm]
#define MOTOR_STEP 			(THREAD_PITCH / (ENCODER_CPR * GEAR_RATIO))
#define MOTOR_MAX_RPS 		(370.0f / 60.0f) // Max rotations per seconds at no load (12V)
#define MOTOR_MAX_FEEDRATE 	(MOTOR_MAX_RPS * THREAD_PITCH) // [mm/s]
// Servo (brush)
#define SERVO_FREQUENCY 	(50.0f) // [Hz]
#define SERVO_DUTY_RANGE 	1000 // Duty cycle range
#define SERVO_TIME_RANGE	(1000.0f / SERVO_FREQUENCY) // [ms]
#define SERVO_TO_DUTY(X)	((X / SERVO_TIME_RANGE) * SERVO_DUTY_RANGE)
#define SERVO_RIGHT 		SERVO_TO_DUTY(1.0f) // [ms]
#define SERVO_CENTER		SERVO_TO_DUTY(1.5f) // [ms]
#define SERVO_LEFT 			SERVO_TO_DUTY(2.0f) // [ms]
// PID regulator
#define KP (0.15f)
#define KI (0.00001f)
#define KD (0.00005f)
#define ALLOWED_ERROR_MARGIN (0.05f)


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
				.moving = false
		},
		.axis_y = {
				.last_sample_time = 0,
				.move_start_time = 0,
				.last_position = 0.0,
				.move_start = 0.0,
				.move_end = 0.0,
				.last_error = 0.0,
				.cumulative_error = 0.0,
				.moving = false
		},
		.feedrate = 0.0,
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

void jog(vector_2d_t feedrate) {
	if (feedrate.x > 0.0) {
		_motor_a_timer->Instance->CCR1 = _feedrate_to_duty_cycle(fabsf(feedrate.x));
		_motor_a_timer->Instance->CCR2 = 0;

	} else if (feedrate.x < 0.0) {
		_motor_a_timer->Instance->CCR1 = 0;
		_motor_a_timer->Instance->CCR2 = _feedrate_to_duty_cycle(fabsf(feedrate.x));

	} else if (feedrate.x == 0.0) {
		_motor_a_timer->Instance->CCR1 = 0;
		_motor_a_timer->Instance->CCR2 = 0;

	} else if (feedrate.y > 0.0) {
		_motor_b_timer->Instance->CCR1 = _feedrate_to_duty_cycle(fabsf(feedrate.y));
		_motor_b_timer->Instance->CCR2 = 0;

	} else if (feedrate.y < 0.0) {
		_motor_b_timer->Instance->CCR1 = 0;
		_motor_b_timer->Instance->CCR2 = _feedrate_to_duty_cycle(fabsf(feedrate.y));

	} else if (feedrate.y == 0.0) {
		_motor_b_timer->Instance->CCR1 = 0;
		_motor_b_timer->Instance->CCR2 = 0;
	}
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
			moving = &_state.axis_y.moving;
			break;

		default:
			return;
	}

	// Check if the axis is active
	if (!*moving) {
		return;
	}

	// Check if the position was already reached
	float margin = end_position - current_position;
	if (margin <= ALLOWED_ERROR_MARGIN
			&& margin >= -ALLOWED_ERROR_MARGIN) {
		// Notify about the finished move
		if ((*moving && !_state.axis_x.moving) ||
				(*moving && !_state.axis_y.moving)) {
			printf(COMMAND_FINISHED_REPLY);
		}
		*moving = false;
		_stop_axis(axis, false);
		return;
	}

	// Calculate the axis position difference after move
	float relative_position = end_position - start_position;
	// Calculate the position error
	float error;

	if (relative_position == 0) {
		// Apply simple step
		error = end_position - current_position;

	} else {
		// Apply S-curve
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

	if (_state.position_mode == POSITIONING_ABSOLUTE) {
		_state.axis_x.move_end = position;
	} else {
		_state.axis_x.move_end += position;
	}

	_state.axis_x.move_start = _state.position.x;
	_state.axis_x.moving = true;
	_state.feedrate = feedrate;
}

void move_y(float position, float feedrate) {
	_state.axis_y.move_start_time = HAL_GetTick();

	if (_state.position_mode == POSITIONING_ABSOLUTE) {
		_state.axis_y.move_end = position;
	} else {
		_state.axis_y.move_end += position;
	}

	_state.axis_y.move_start = _state.position.y;
	_state.axis_y.moving = true;
	_state.feedrate = feedrate;
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
			}
			return;

		default:
			return;
	}
}

void stop_x(void) {
	_stop_axis(AXIS_X, true);
}

void stop_y(void) {
	_stop_axis(AXIS_Y, true);
}

void stop(void) {
	_stop_axis(AXIS_X, true);
	_stop_axis(AXIS_Y, true);
	printf(COMMAND_FINISHED_REPLY);
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
	printf(COMMAND_FINISHED_REPLY);
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
