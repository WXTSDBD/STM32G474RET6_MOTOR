/**
 * @file motor_current.h
 * @date 2026-10-06
 * @brief M1 电流环对外入口：初始化、20 kHz 节拍、模式与电流指令。
 *
 * 调用方只通过本头改模式和指令。Park 角从哪来、PI 怎么算，不在本头展开。
 * motor_current_tick() 只允许从 ADC2 注入转换完成中断调用
 * （HAL_ADCEx_InjectedConvCpltCallback）。禁止在任务里调用，会与本拍重入。
 * 外环读写函数给 2 kHz 任务用；不要从那里再进 tick。
 *
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
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

void motor_current_set_idq_ref(bsp_axis_t *axis, float id_ref, float iq_ref);

void motor_startup_arm_axis(bsp_axis_t *axis);

motor_context_t *motor_current_ctx(const bsp_axis_t *axis);

float motor_current_get_pll_omega_mech_rpm(void);

float motor_current_get_enc_pll_omega_mech_rpm(void);

float motor_current_get_theta_mech_rad(void);

float motor_current_get_theta_fb_rad(void);

void motor_current_pll_reset_now(void);

#if M1_SPEED_LOOP_ENABLE
void motor_current_outer_set_mode(bsp_axis_t *axis, m1_outer_mode_t mode);
void motor_current_set_omega_ref_rpm(bsp_axis_t *axis, float rpm);
void motor_current_set_theta_ref_rad(bsp_axis_t *axis, float theta_rad);
void motor_current_arm_position_hold(bsp_axis_t *axis);
void motor_current_set_iq_cmd(bsp_axis_t *axis, float iq_a);
#endif

#ifdef __cplusplus
}
#endif

#endif
