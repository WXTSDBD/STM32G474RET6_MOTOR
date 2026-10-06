/**
 * @file deadband_flow.h
 * @date 2026-10-06
 * @brief 死区标定和辨识配方调度。

 *
 * tick 只允许从电流环节拍调用。boot 给上电一次。
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
 */

#ifndef DEADBAND_FLOW_H
#define DEADBAND_FLOW_H

#include "motor_context.h"
#include "motor_params_m1.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 上电/复位：进入配方第一步（Id 标定或 ident HOLD） */
void deadband_flow_boot(motor_context_t *ctx);

/** 20 kHz：推进当前配方步 */
void deadband_flow_tick(motor_context_t *ctx);

/**
 * Id 标定是否仍占用热路径（PI 限幅 / capture / align Ud）。
 * ident 段或标定 DONE 后返回 0。
 */
uint8_t deadband_flow_id_cal_active(void);

/**
 * Id→ident 切换后需重配 PI 限幅时置 1（M1_IDENT_OVERRIDE_LIMITS）。
 * motor_current 读后清零。
 */
uint8_t deadband_flow_consume_ident_pi_retune(void);

#if M1_IDENT_ENABLE
uint8_t deadband_flow_in_ident(void);
#else
static inline uint8_t deadband_flow_in_ident(void) { return 0u; }
#endif

#if M1_DEADBAND_FLOW_ONE_SHOT && M1_SPEED_LOOP_ENABLE
/** ONE_SHOT 段②③：速度阶梯，非 Id 标定 */
uint8_t deadband_flow_speed_ladder_active(void);
#else
static inline uint8_t deadband_flow_speed_ladder_active(void) { return 0u; }
#endif

#if M1_SPEED_IDENT_ENABLE
/** 速度环阶跃/Bode 段：deadband OFF，ω_ref 由 speed_ident 产生 */
uint8_t deadband_flow_speed_ident_active(void);
#else
static inline uint8_t deadband_flow_speed_ident_active(void) { return 0u; }
#endif

#ifdef __cplusplus
}
#endif

#endif /* DEADBAND_FLOW_H */
