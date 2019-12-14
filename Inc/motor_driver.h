/*
 * motor_driver.h
 *
 *  Created on: 14. okt. 2019
 *      Author: adamh
 */

#ifndef MOTOR_DRIVER_H_
#define MOTOR_DRIVER_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32l4xx_hal.h"

/**
 * Movement axis definition
 */
typedef enum Axes {
	AXIS_X,
	AXIS_Y
} axis_t;

/**
 * Vector 2D
 */
typedef struct Vector2D {
	float x;
	float y;
} vector_2d_t;

/**
 * Axis information
 */
typedef struct AxisInfo {
	uint32_t last_sample_time;
	uint32_t move_start_time;
	float last_position;
	float move_start;
	float move_end;
	float last_error;
	float cumulative_error;
} axis_info_t;

/**
 * Current driver state
 */
typedef struct MachineState {
	vector_2d_t position;
	uint8_t z;
	axis_info_t axis_x;
	axis_info_t axis_y;
	float feedrate;
} state_t;

/**
 * Position encoder tracker
 */
typedef struct PositionEncoder {
	int16_t old_encoder_count;
	int32_t encoder_count_absolute;
} position_encoder_t;

/**
 * Initialize the motor driver
 */
void motor_driver_init(
		TIM_HandleTypeDef *motor_a_timer,
		TIM_HandleTypeDef *motor_a_encoder_timer,
		TIM_HandleTypeDef *motor_b_timer,
		TIM_HandleTypeDef *motor_b_encoder_timer);

/**
 * Enable the X axis - motor A
 */
void axis_x_enable(void);

/**
 * Disable the X axis - motor A
 */
void axis_x_disable(void);

/**
 * Enable the Y axis - motor B
 */
void axis_y_enable(void);

/**
 * Disable the Y axis - motor B
 */
void axis_y_disable(void);

/**
 * Move the tool absolutely on the X axis
 *
 * @param position	The position on which to move the X axis
 * @param feedrate	Feedrate at which to move the X axis
 */
void move_x(float position, float feedrate);

/**
 * Move the tool absolutely on the Y axis
 *
 * @param position	The position on which to move the Y axis
 * @param feedrate	Feedrate at which to move the Y axis
 */
void move_y(float position, float feedrate);

//void move_z(float position);

/**
 * Move the tool to the specified position on all axes
 *
 * @param position 	Vector2D position (X, Y) at which to move the tool
 * @param feedrate	Feedrate at which to move the tool
 */
void move(vector_2d_t position, float feedrate);

/**
 * Stop the X axis
 */
void stop_x(void);

/**
 * Stop the Y axis
 */
void stop_y(void);

/**
 * Stop all axis
 */
void stop(void);

/**
 * Move the tool on the X axis to home
 */
void home_x(void);

/**
 * Move the tool on the Y axis to home
 */
void home_y(void);

/**
 * Move the tool on both axes to home
 */
void home(void);

/**
 * Update motor controls and read their absolute positions
 */
void motor_driver_update(void);

/**
 * Get the current motor driver state
 *
 * @returns	Current motor driver state
 */
state_t get_state(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_DRIVER_H_ */
