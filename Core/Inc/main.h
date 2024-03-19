/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "fonts.h"
#include "ILI9341_GFX.h"
#include "xpt2046_touch.h"
#include "stdio.h"
#include "string.h"
#include "INA219.h"
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
#define TOUCH_CS_Pin GPIO_PIN_0
#define TOUCH_CS_GPIO_Port GPIOB
#define TFT_CS_Pin GPIO_PIN_1
#define TFT_CS_GPIO_Port GPIOB
#define IRQ_Pin GPIO_PIN_2
#define IRQ_GPIO_Port GPIOB
#define TFT_RST_Pin GPIO_PIN_10
#define TFT_RST_GPIO_Port GPIOB
#define TFT_DC_Pin GPIO_PIN_11
#define TFT_DC_GPIO_Port GPIOB

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define TOUCH_CS_Pin GPIO_PIN_0
#define TOUCH_CS_GPIO_Port GPIOB
#define TFT_CS_Pin GPIO_PIN_1
#define TFT_CS_GPIO_Port GPIOB
#define TFT_RST_Pin GPIO_PIN_10
#define TFT_RST_GPIO_Port GPIOB
#define TFT_DC_Pin GPIO_PIN_11
#define TFT_DC_GPIO_Port GPIOB
#define HC595_ST_CP1_Pin GPIO_PIN_12
#define HC595_ST_CP1_GPIO_Port GPIOB
#define HC595_ST_CP5_Pin GPIO_PIN_14
#define HC595_ST_CP5_GPIO_Port GPIOB
#define HC595_MR_Pin GPIO_PIN_8
#define HC595_MR_GPIO_Port GPIOA
#define HC595_ST_CP4_Pin GPIO_PIN_3
#define HC595_ST_CP4_GPIO_Port GPIOB
#define HC595_ST_CP3_Pin GPIO_PIN_4
#define HC595_ST_CP3_GPIO_Port GPIOB
#define HC595_ST_CP2_Pin GPIO_PIN_8
#define HC595_ST_CP2_GPIO_Port GPIOB
#define HC595_ST_CP6_Pin GPIO_PIN_9
#define HC595_ST_CP6_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
