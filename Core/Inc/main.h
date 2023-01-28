/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32g4xx_hal.h"

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
#define HRTIM_INPUT_CLOCK 170000000
#define HRTIMA_PWM_FREQ 10000
#define MAX_PWM_DUTY_CYCLE (HRTIMA_PERIOD/100)*60
#define HRTIMA_PERIOD ((uint16_t)(((uint64_t)HRTIM_INPUT_CLOCK/8)/HRTIMA_PWM_FREQ))
#define DIN_5_Pin GPIO_PIN_13
#define DIN_5_GPIO_Port GPIOC
#define DIN_6_Pin GPIO_PIN_14
#define DIN_6_GPIO_Port GPIOC
#define DIN_7_Pin GPIO_PIN_15
#define DIN_7_GPIO_Port GPIOC
#define BUTTON_1_Pin GPIO_PIN_0
#define BUTTON_1_GPIO_Port GPIOC
#define LED_1_Pin GPIO_PIN_1
#define LED_1_GPIO_Port GPIOC
#define LED_2_Pin GPIO_PIN_2
#define LED_2_GPIO_Port GPIOC
#define LED_3_Pin GPIO_PIN_3
#define LED_3_GPIO_Port GPIOC
#define LED_4_Pin GPIO_PIN_14
#define LED_4_GPIO_Port GPIOB
#define LED_5_Pin GPIO_PIN_15
#define LED_5_GPIO_Port GPIOB
#define RELAY_3_Pin GPIO_PIN_11
#define RELAY_3_GPIO_Port GPIOC
#define RELAY_2_Pin GPIO_PIN_12
#define RELAY_2_GPIO_Port GPIOC
#define RELAY_1_Pin GPIO_PIN_2
#define RELAY_1_GPIO_Port GPIOD
#define DIN_1_Pin GPIO_PIN_3
#define DIN_1_GPIO_Port GPIOB
#define DIN_2_Pin GPIO_PIN_4
#define DIN_2_GPIO_Port GPIOB
#define DIN_3_Pin GPIO_PIN_5
#define DIN_3_GPIO_Port GPIOB
#define DIN_4_Pin GPIO_PIN_6
#define DIN_4_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
