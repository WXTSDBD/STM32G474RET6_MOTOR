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

/** PLL 机械转速 [rpm]（20 kHz 最新值，供 2 kHz 外环读） */
float motor_current_get_pll_omega_mech_rpm(void);

/** unwrap 机械角 [rad]（20 kHz 缓存，多圈连续，上电相对零） */
float motor_current_get_theta_mech_rad(void);

/** 速度 ident / 切换前：θ←编码器、ω←0，清 PLL 积分 */
void motor_current_pll_reset_now(void);

#if M1_SPEED_LOOP_ENABLE
void motor_current_outer_set_mode(bsp_axis_t *axis, m1_outer_mode_t mode);
void motor_current_set_omega_ref_rpm(bsp_axis_t *axis, float rpm);
void motor_current_set_theta_ref_rad(bsp_axis_t *axis, float theta_rad);
/** θ_ref ← 当前 θ_mech，不改变 outer_mode */
void motor_current_arm_position_hold(bsp_axis_t *axis);
void motor_current_set_iq_cmd(bsp_axis_t *axis, float iq_a);
#endif

#ifdef __cplusplus
}
#endif

#endif
