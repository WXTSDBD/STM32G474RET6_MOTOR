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

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
/** Debug monitor: Watch ???? dbg ???????? */
typedef struct {
  uint32_t csr;
  uint8_t en;
  uint8_t intout;
  uint16_t pggain;
  uint16_t adc_jdr1;
  uint32_t adc_irq_cnt;
  uint32_t hal_state;   /* HAL_OPAMP_StateTypeDef, READY=1 BUSY=2 */
} DbgOpampChan_t;

typedef struct {
  DbgOpampChan_t opamp[3]; /* [0]=OPAMP1/ADC1 [1]=OPAMP3/ADC3 [2]=OPAMP4/ADC5 */
  int16_t adc_shunt[3];      /* ? adc_read[0..2] ?? */
  int16_t adc_reg[3];        /* ? adc_read[3..5] ??(ADC2) */
  int32_t adc_offset[3];     /* M1 ???????? */
  int16_t adc_zeroed[3];     /* raw[i] - offset[i]?LSB? */
  float adc_ia;              /* M1 ????A? */
  float adc_ib;
  float adc_ic;
  float foc_theta_el;        /* ?? Park/SVPWM ????rad? */
  float foc_id;              /* ???? dq ???A? */
  float foc_iq;
  float enc_cal_add;         /* final add after pi offset (rad) */
  float enc_cal_add_raw;     /* lock-rotor raw add before pi (rad) */
} DbgMon_t;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/** 1=???????? add?0 ? M1_ENCODER_OFFSET_RAD?????? */
#define M1_RUN_ENCODER_CAL 0

#ifndef M1_ENCODER_OFFSET_RAD
#define M1_ENCODER_OFFSET_RAD 6.0f
#endif

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
extern volatile DbgMon_t dbg;
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
