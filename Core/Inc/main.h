/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "stm32f1xx_hal.h"

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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define AIN1_Pin GPIO_PIN_0
#define AIN1_GPIO_Port GPIOA
#define AIN2_Pin GPIO_PIN_1
#define AIN2_GPIO_Port GPIOA
#define BIN1_Pin GPIO_PIN_2
#define BIN1_GPIO_Port GPIOA
#define BIN2_Pin GPIO_PIN_3
#define BIN2_GPIO_Port GPIOA
#define A_A_Pin GPIO_PIN_6
#define A_A_GPIO_Port GPIOA
#define A_B_Pin GPIO_PIN_7
#define A_B_GPIO_Port GPIOA
#define A_B_EXTI_IRQn EXTI9_5_IRQn
#define RGB_BLUE_Pin GPIO_PIN_12
#define RGB_BLUE_GPIO_Port GPIOB
#define RGB_RED_Pin GPIO_PIN_13
#define RGB_RED_GPIO_Port GPIOB
#define RGB_GREEN_Pin GPIO_PIN_14
#define RGB_GREEN_GPIO_Port GPIOB
#define KEY2_Pin GPIO_PIN_8
#define KEY2_GPIO_Port GPIOA
#define KEY2_EXTI_IRQn EXTI9_5_IRQn
#define ESP01S_TX_Pin GPIO_PIN_9
#define ESP01S_TX_GPIO_Port GPIOA
#define ESP01S_RX_Pin GPIO_PIN_10
#define ESP01S_RX_GPIO_Port GPIOA
#define KEY3_Pin GPIO_PIN_11
#define KEY3_GPIO_Port GPIOA
#define KEY3_EXTI_IRQn EXTI15_10_IRQn
#define KEY1_Pin GPIO_PIN_12
#define KEY1_GPIO_Port GPIOA
#define KEY1_EXTI_IRQn EXTI15_10_IRQn
#define B_A_Pin GPIO_PIN_6
#define B_A_GPIO_Port GPIOB
#define B_A_EXTI_IRQn EXTI9_5_IRQn
#define B_B_Pin GPIO_PIN_7
#define B_B_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
