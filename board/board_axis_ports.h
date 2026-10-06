/**
 * @file board_axis_ports.h
 * @date 2026-10-06
 * @brief M1/M2 的 PWM 口和 ADC 口实例。

 *
 * bind_hal 由 bridge 在上电调用。
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
 */

#ifndef BOARD_AXIS_PORTS_H
#define BOARD_AXIS_PORTS_H

#include "adc_foc_port.h"
#include "bsp_axes.h"
#include "pwm_port.h"

adc_foc_port_t *board_adc_foc_port(bsp_axis_id_t id);
pwm_port_t *board_pwm_port(bsp_axis_id_t id);

/** 将 CubeMX HAL 句柄写入 port->hw（由 bridge_cubemx 调用）。 */
void board_axis_ports_bind_hal(bsp_axis_id_t id);

#endif
