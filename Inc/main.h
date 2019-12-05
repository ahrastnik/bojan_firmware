/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define EN_B_Pin GPIO_PIN_1
#define EN_B_GPIO_Port GPIOC
#define HOME_X_Pin GPIO_PIN_2
#define HOME_X_GPIO_Port GPIOC
#define HOME_Y_Pin GPIO_PIN_3
#define HOME_Y_GPIO_Port GPIOC
#define IN1_B_Pin GPIO_PIN_0
#define IN1_B_GPIO_Port GPIOA
#define IN2_B_Pin GPIO_PIN_1
#define IN2_B_GPIO_Port GPIOA
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define SERVO_Pin GPIO_PIN_6
#define SERVO_GPIO_Port GPIOA
#define R_Pin GPIO_PIN_6
#define R_GPIO_Port GPIOC
#define G_Pin GPIO_PIN_7
#define G_GPIO_Port GPIOC
#define B_Pin GPIO_PIN_8
#define B_GPIO_Port GPIOC
#define ENC1_A_Pin GPIO_PIN_8
#define ENC1_A_GPIO_Port GPIOA
#define ENC2_A_Pin GPIO_PIN_9
#define ENC2_A_GPIO_Port GPIOA
#define EN_A_Pin GPIO_PIN_10
#define EN_A_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define IN1_A_Pin GPIO_PIN_4
#define IN1_A_GPIO_Port GPIOB
#define IN2_A_Pin GPIO_PIN_5
#define IN2_A_GPIO_Port GPIOB
#define ENC1_B_Pin GPIO_PIN_6
#define ENC1_B_GPIO_Port GPIOB
#define ENC2_B_Pin GPIO_PIN_7
#define ENC2_B_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
