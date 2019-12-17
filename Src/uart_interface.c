/*
 * uart_interface.c
 *
 *  Created on: 13. okt. 2019
 *      Author: adamh
 */

#include "uart_interface.h"

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <ctype.h>
#include <errno.h>

#include "motor_driver.h"

#define COMMAND_LEN_MAX     32
#define COMMAND_LEN_MIN     2
#define CMD_DELIMITER       " "
#define COMMAND_ARGS_MAX	8
#define ARGS_LEN_MAX		32

#define DEFAULT_G00_FEEDRATE	(10.0f)
#define DEFAULT_G01_FEEDRATE	(5.0f)

/**
 * Converts a string argument to a number
 *
 * @param	str		String to convert
 */
static long str_to_num(char *str);
/**
 * Converts a string argument to a float
 *
 * @param	str		String to convert
 */
static float str_to_float(char *str);
/**
 * Command handler
 * Extracts, validates and executes received commands.
 *
 * @param	command		Command to handle
 */
static void handle_command(command_t command);

/**
 * Command list
 *
 * Note - The Commands(and their order) in this list must EXACTLY MATCH
 * the commands in the "command_t" enumerator.
 */
const char* COMMANDS[] = {
    "RESET",
    "LED",
	"M112",
	"M114",
	"G00",
	"G01",
	"G28",
	"G90",
	"G91",
    NULL
};
// Command buffer
static uint8_t command_byte_buffer;
static uint8_t command_buffer[COMMAND_LEN_MAX];
static uint8_t command_length = 0;
static char command_args[COMMAND_ARGS_MAX][ARGS_LEN_MAX];
static uint8_t command_args_num = 0;

void uart_interface_init(UART_HandleTypeDef *huart) {
	HAL_UART_Receive_DMA(huart, &command_byte_buffer, 1);
}

// Overload the UART RX callback
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	// Check for command buffer overflow
	if (command_length >= COMMAND_LEN_MAX) {
		// Reset the command buffer
		command_length = 0;
		return;
	}
	// Buffer the received byte
	command_buffer[command_length++] = command_byte_buffer;

	// Ignore, if the command hasn't reached minimal length
	if (command_length < COMMAND_LEN_MIN) {
		return;
	}
	// Check for CR/LF termination
	if (command_buffer[command_length-2] != '\r'
			|| command_buffer[command_length-1] != '\n') {
		return;
	}
	// Move the null termination to the start of CR/LF
	command_buffer[command_length-2] = '\0';
	// Convert the command to lower case
	uint16_t i;
	for(i = 0; command_buffer[i]; i++) {
		command_buffer[i] = toupper(command_buffer[i]);
	}

	// Extract the command
	i = 0;
	int16_t cmd_index = -1;
	char *cmd = strtok((char *)command_buffer, CMD_DELIMITER);
	// Check if the received string is a command
	while (COMMANDS[i] != NULL) {
		if (strcmp(cmd, COMMANDS[i]) == 0) {
			cmd_index = i;
			break;
		}
		i++;
	}

	// Get arguments, if they exist
	char *args = strtok(NULL, CMD_DELIMITER);
	while (args != NULL) {
		uint32_t arg_len = strlen(args);
		if (arg_len > (ARGS_LEN_MAX - 1)) {
			// Skip argument, if the it's longer, than maximum allowed length
			continue;
		}
		memcpy(command_args[command_args_num++], args, arg_len+1);
		// Get the next argument
		args = strtok(NULL, CMD_DELIMITER);
	}

	// Run the command
	handle_command((command_t)cmd_index);
	// Reset the command buffer
	command_length = 0;
	// Reset argument buffer
	command_args_num = 0;
}

static long str_to_num(char *str) {
    long number = strtol(str, NULL, 10);
    if (number == 0 && (errno == EINVAL || errno == ERANGE)) {
        //ESP_LOGE(TAG, "The given string can't be converted to an integer!");
        errno = 0;
    }
    return number;
}

static float str_to_float(char *str) {
	float number = strtof(str, NULL);
	if (number == 0 && (errno == EINVAL || errno == ERANGE)) {
		//ESP_LOGE(TAG, "The given string can't be converted to a float!");
		errno = 0;
	}
	return number;
}

static void handle_command(command_t command) {
	state_t driver_state = get_state();

	switch (command) {
		case CMD_RESET:
			// Reset the system
			printf("Reseting...\n");
			NVIC_SystemReset();
			break;

		case CMD_LED:
			// Toggle the onboard LED
			if (command_args_num == 1) {
				if (strcmp(command_args[0], "on") == 0) {
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
				} else if (strcmp(command_args[0], "off") == 0) {
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
				}
				break;
			}
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
			break;

		case CMD_M112:
			// Emergency stop
			if (command_args_num == 1) {
				if (command_args[0][0] == 'X') {
					stop_x();

				} else if (command_args[0][0] == 'Y') {
					stop_y();

				} else {
					stop();
				}
				break;
			}
			stop();
			break;

		case CMD_M114:
			printf("X: %f\tY: %f\n", driver_state.position.x, driver_state.position.y);
			break;

		case CMD_G00:
			// Rapid positioning
			if (command_args_num >= 1) {
				driver_state.feedrate = DEFAULT_G00_FEEDRATE;
				char parse_buffer[COMMAND_ARGS_MAX];

				// Parse X coordinate
				uint8_t i;
				for (i = 0; i < command_args_num; i++) {
					memcpy(parse_buffer, (command_args[i])+1, strlen(command_args[i]));
					if (command_args[i][0] == 'X') {
						driver_state.position.x = str_to_float(parse_buffer);

					} else if (command_args[i][0] == 'Y') {
						driver_state.position.y = str_to_float(parse_buffer);

					} else if (command_args[i][0] == 'Z') {
						if (str_to_float(parse_buffer) > 0) {
							brush_drop();
						} else {
							brush_raise();
						}

					} else if (command_args[i][0] == 'F') {
						driver_state.feedrate = str_to_float(parse_buffer);
					}
				}
				move(driver_state.position, driver_state.feedrate);
			}
			break;

		case CMD_G01:
			// Linear interpolation
			break;

		case CMD_G28:
			// Home axes
			break;

		case CMD_G90:
			// Absolute positioning
			positioning_absolute();
			break;

		case CMD_G91:
			// Relative positioning
			positioning_relative();
			break;

		default:
			printf("Invalid command!\n");
			break;
	}
}
