/**
 * @file motor_current.h
 * @brief M1 20 kHz 电流环节拍（ADC2 JEOC）。
 */

#ifndef MOTOR_CURRENT_H
#define MOTOR_CURRENT_H

#include "bsp_axes.h"
#include "motor_context.h"

#ifdef __cplusplus
extern "C" {
#endif

void motor_current_init(bsp_axis_t *axis);
void motor_current_tick(bsp_axis_t *axis);
void motor_current_set_mode(bsp_axis_t *axis, m1_ctrl_mode_t mode);

/** 设置 dq 电流指令（A）；受 M1_I_REF_ABS_MAX 钳位；启动完成后由 ramp 跟踪 */
void motor_current_set_idq_ref(bsp_axis_t *axis, float id_ref, float iq_ref);

/** 重新从 ALIGN 跑启动序列（CURRENT_LOOP 下） */
void motor_startup_arm_axis(bsp_axis_t *axis);

motor_context_t *motor_current_ctx(const bsp_axis_t *axis);

#ifdef __cplusplus
}
#endif

#endif
