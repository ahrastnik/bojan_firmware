/*
 * uart_interface.h
 *
 *  Created on: 13. okt. 2019
 *      Author: adamh
 */

#ifndef UART_INTERFACE_H_
#define UART_INTERFACE_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32l4xx_hal.h"

/**
 * List of available commands
 */
typedef enum {
    CMD_RESET,
    CMD_LED,
    CMD_NUM
} command_t;

/**
 * Initialize the UART command interface
 *
 * @param 	huart		UART handle
 */
void uart_interface_init(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif /* UART_INTERFACE_H_ */
