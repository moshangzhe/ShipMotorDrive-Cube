/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32g0xx_hal.h"

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
#define S1_Pin GPIO_PIN_0
#define S1_GPIO_Port GPIOA
#define S1_EXTI_IRQn EXTI0_1_IRQn
#define S2_Pin GPIO_PIN_1
#define S2_GPIO_Port GPIOA
#define S2_EXTI_IRQn EXTI0_1_IRQn
#define S3_Pin GPIO_PIN_2
#define S3_GPIO_Port GPIOA
#define S3_EXTI_IRQn EXTI2_3_IRQn
#define S4_Pin GPIO_PIN_3
#define S4_GPIO_Port GPIOA
#define S4_EXTI_IRQn EXTI2_3_IRQn
#define M2B_Pin GPIO_PIN_4
#define M2B_GPIO_Port GPIOA
#define M2A_Pin GPIO_PIN_5
#define M2A_GPIO_Port GPIOA
#define M1B_Pin GPIO_PIN_6
#define M1B_GPIO_Port GPIOA
#define M1A_Pin GPIO_PIN_7
#define M1A_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
