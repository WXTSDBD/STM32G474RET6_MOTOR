/**
 * @file hal_bridge.h
 * @brief CubeMX 生成的 HAL 句柄集中声明，供 config/、board/、platform/ 使用。
 *
 * motor/ 不得 include 本文件。句柄定义仍由 Core/Inc 外设头与 CubeMX 生成。
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
