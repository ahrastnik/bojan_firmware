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

#define COMMAND_LEN_MAX     32
#define COMMAND_LEN_MIN     2
#define CMD_DELIMITER       " "
#define COMMAND_ARGS_MAX	8
#define ARGS_LEN_MAX		32

/**
 * Converts a string argument to a number
 *
 * @param	str		String to convert
 */
static long str_to_num(char *str);
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
    "reset",
    "led",
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
		command_buffer[i] = tolower(command_buffer[i]);
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

static void handle_command(command_t command) {
	switch (command) {
	case CMD_RESET:
		printf("Reseting...\n");
		NVIC_SystemReset();
		break;

	case CMD_LED:
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

	default:
		printf("Invalid command!\n");
		break;
	}
}
