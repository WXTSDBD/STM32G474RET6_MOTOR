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
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "dbg_monitor.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
/* DbgMon_t �?? debug/dbg_monitor.h */
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
/* 台架�?关真相源：config/bringup_bench.h（BINDING_OVERRIDE 默认 0）�?? */
#include "bringup_bench.h"
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
/* extern dbg �?? debug/dbg_monitor.h */
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define I2C1_CRL_Pin GPIO_PIN_13
#define I2C1_CRL_GPIO_Port GPIOC
#define KEY1_Pin GPIO_PIN_14
#define KEY1_GPIO_Port GPIOC
#define KEY2_Pin GPIO_PIN_15
#define KEY2_GPIO_Port GPIOC
#define LED1_Pin GPIO_PIN_2
#define LED1_GPIO_Port GPIOC
#define NTC_MOS1_Pin GPIO_PIN_2
#define NTC_MOS1_GPIO_Port GPIOA
#define SPI1_CS_Pin GPIO_PIN_4
#define SPI1_CS_GPIO_Port GPIOA
#define LED2_Pin GPIO_PIN_5
#define LED2_GPIO_Port GPIOC
#define VBUS_Pin GPIO_PIN_1
#define VBUS_GPIO_Port GPIOB
#define NTC_MOS2_Pin GPIO_PIN_12
#define NTC_MOS2_GPIO_Port GPIOB
#define LED3_Pin GPIO_PIN_9
#define LED3_GPIO_Port GPIOC
#define SPI3_CS_Pin GPIO_PIN_15
#define SPI3_CS_GPIO_Port GPIOA
#define SPI3_FLASH_CS_Pin GPIO_PIN_2
#define SPI3_FLASH_CS_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
