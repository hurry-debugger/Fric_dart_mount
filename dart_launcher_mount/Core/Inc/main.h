/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

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
#define Fric_L_1_Pin GPIO_PIN_5
#define Fric_L_1_GPIO_Port GPIOE
#define Fric_L_2_Pin GPIO_PIN_6
#define Fric_L_2_GPIO_Port GPIOE
#define TOF_UART_TX_Pin GPIO_PIN_0
#define TOF_UART_TX_GPIO_Port GPIOA
#define DEBUG_UART_TX_Pin GPIO_PIN_2
#define DEBUG_UART_TX_GPIO_Port GPIOA
#define DEBUG_UART_RX_Pin GPIO_PIN_3
#define DEBUG_UART_RX_GPIO_Port GPIOA
#define Laser_control_Pin GPIO_PIN_12
#define Laser_control_GPIO_Port GPIOE
#define JUDGE_UART_TX_Pin GPIO_PIN_10
#define JUDGE_UART_TX_GPIO_Port GPIOB
#define JUDGE_UART_RX_Pin GPIO_PIN_11
#define JUDGE_UART_RX_GPIO_Port GPIOB
#define Fric_R_1_Pin GPIO_PIN_14
#define Fric_R_1_GPIO_Port GPIOB
#define Fric_R_2_Pin GPIO_PIN_15
#define Fric_R_2_GPIO_Port GPIOB
#define RELOAD_INIT_Pin GPIO_PIN_14
#define RELOAD_INIT_GPIO_Port GPIOD
#define YAW_INIT_Pin GPIO_PIN_15
#define YAW_INIT_GPIO_Port GPIOD
#define DBUS_UART_TX_Pin GPIO_PIN_9
#define DBUS_UART_TX_GPIO_Port GPIOA
#define DBUS_UART_RX_Pin GPIO_PIN_10
#define DBUS_UART_RX_GPIO_Port GPIOA
#define TOF_UART_RX_Pin GPIO_PIN_11
#define TOF_UART_RX_GPIO_Port GPIOC
#define OLED_UART_TX_Pin GPIO_PIN_12
#define OLED_UART_TX_GPIO_Port GPIOC
#define OLED_UART_RX_Pin GPIO_PIN_2
#define OLED_UART_RX_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
