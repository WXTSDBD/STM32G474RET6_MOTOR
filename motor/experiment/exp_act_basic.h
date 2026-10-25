/**
 * @file exp_act_basic.h
 * @date 2026-10-09
 * @brief SCHEDULE 内建动作：HOLD / 速度给定（E3）。
 *
 * 段表 act 填 EXP_ACT_* 且 ops=NULL 时由 runner 选用这些 ops。
 */

#ifndef MOTOR_EXPERIMENT_EXP_ACT_BASIC_H
#define MOTOR_EXPERIMENT_EXP_ACT_BASIC_H

#include "exp_def.h"

#ifdef __cplusplus
extern "C" {
#endif

extern const exp_action_ops_t g_exp_act_hold;
extern const exp_action_ops_t g_exp_act_omega_ref;
/** 固定 Park 角 + d 轴电流注入（Ud/Id 锁轴标定用）：p0=电角(deg)，p1=Id(A)。 */
extern const exp_action_ops_t g_exp_act_theta_ref;

const exp_action_ops_t *exp_act_basic_lookup(uint8_t act);

/** @brief 固定角注入是否已生效（进入 THETA_REF 段后置位）。 */
uint8_t exp_act_theta_is_active(void);
/** @brief 取当前固定电角，单位 rad。 */
float exp_act_theta_get_rad(void);
/** @brief 取当前注入的 Id，单位 A。 */
float exp_act_theta_get_id_a(void);
/** @brief 清除固定角注入标志（实验结束/未武装时调用）。 */
void exp_act_theta_clear(void);
/* 1 = table-driven experiment already set the angle source */
uint8_t exp_act_angle_src_is_active(void);
/* angle source: 0=observer (sensorless), 1=encoder (sensored) */
uint8_t exp_act_angle_src(void);
void exp_act_angle_from_mark(uint8_t mark);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_EXPERIMENT_EXP_ACT_BASIC_H */
