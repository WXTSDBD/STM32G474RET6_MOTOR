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
/* DbgMon_t �? debug/dbg_monitor.h */
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/** 1=上电锁转子标 encoder add�??0=�?? M1_ENCODER_OFFSET_RAD */
#define M1_RUN_ENCODER_CAL 0

/** 1=上电跑三档脉冲诊断（不写 Flash，标�?? hold）；0=�?? Flash binding */
#define M1_RUN_PHASE_CAL 0

/** 1=�?? Flash 加载 phase binding（M1_RUN_PHASE_CAL=0 时生效） */
#define M1_APPLY_PHASE_BINDING 1

/** 1=强制 2325 Test3 binding，忽�?? Flash（开环验收）�??0=仅用 Flash */
#ifndef M1_BINDING_OVERRIDE_2325
#define M1_BINDING_OVERRIDE_2325  1
#endif

#ifndef M1_ENCODER_OFFSET_RAD
#define M1_ENCODER_OFFSET_RAD 2.10f
#endif

/** 1=�?? Park/VOFA �?? -θ（测�?? B）；SVPWM �?? +θ。闭环前须改�?? 0 或统�?? encoder 约定 */
#ifndef M1_THETA_NEGATE
#define M1_THETA_NEGATE  0
#endif

/** 1=VOFA ch4 输出 SVPWM 扇区 1..6（替�?? Id）；验证扇区 vs Iq 后改�?? 0 */
#ifndef M1_VOFA_SECTOR_DIAG
#define M1_VOFA_SECTOR_DIAG  0
#endif

/** 1=VOFA ch0�?2 输出 foc_ia/b/c(A)，与 Park 同拍�?0=adc_zeroed LSB */
#ifndef M1_VOFA_FOC_ABC
#define M1_VOFA_FOC_ABC  1
#endif

#define PHASE_CAL_FAIL_NONE     0u
#define PHASE_CAL_FAIL_OC       1u
#define PHASE_CAL_FAIL_SNR      2u
#define PHASE_CAL_FAIL_PERM     3u
#define PHASE_CAL_FAIL_FLASH    4u
#define PHASE_CAL_FAIL_TIMEOUT  5u

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
/* extern dbg �? debug/dbg_monitor.h */
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
