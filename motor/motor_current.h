/**
 * @file motor_current.h
 * @brief M1 20 kHz 电流环节拍（ADC2 JEOC）。
 */

#ifndef MOTOR_CURRENT_H
#define MOTOR_CURRENT_H

#include "adc.h"
#include "bsp_axes.h"
#include "motor_context.h"

#ifdef __cplusplus
extern "C" {
#endif

void motor_current_init(bsp_axis_t *axis);
void motor_current_tick(bsp_axis_t *axis, ADC_HandleTypeDef *hadc);
void motor_current_set_mode(bsp_axis_t *axis, m1_ctrl_mode_t mode);

motor_context_t *motor_current_ctx(const bsp_axis_t *axis);

#ifdef __cplusplus
}
#endif

#endif
