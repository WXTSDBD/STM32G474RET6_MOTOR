/**
 * @file hal_bridge.h
 * @date 2026-10-06
 * @brief CubeMX HAL 句柄集中声明。

 *
 * motor/ 不要 include 本头。句柄定义仍在 CubeMX 生成文件。
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
 */

#ifndef HAL_BRIDGE_H
#define HAL_BRIDGE_H

#include "stm32g4xx_hal.h"

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;
extern ADC_HandleTypeDef hadc5;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim15;

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi3;

extern CORDIC_HandleTypeDef hcordic;

extern UART_HandleTypeDef hlpuart1;
extern UART_HandleTypeDef huart1;

extern OPAMP_HandleTypeDef hopamp1;
extern OPAMP_HandleTypeDef hopamp3;
extern OPAMP_HandleTypeDef hopamp4;

extern I2C_HandleTypeDef hi2c1;

extern FDCAN_HandleTypeDef hfdcan1;

#endif
