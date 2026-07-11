/**
 * @file motor_context.h
 * @brief M1 控制上下文：开环 / PI 影子 / 电流环（后续）。
 */

#ifndef MOTOR_CONTEXT_H
#define MOTOR_CONTEXT_H

#include <stdint.h>

#include "foc_pi.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    M1_CTRL_OPEN_LOOP = 0,
    M1_CTRL_OBSERVE_ONLY,
    M1_CTRL_CURRENT_LOOP,
} m1_ctrl_mode_t;

/** 2026-06-19 电流环联调：上电即 CURRENT_LOOP；开环死区扫参改 OBSERVE_ONLY */
#ifndef M1_CTRL_MODE_DEFAULT
#define M1_CTRL_MODE_DEFAULT M1_CTRL_CURRENT_LOOP
#endif

/**
 * @brief 外环模式：谁产生 iq_ref。
 *
 * 与 m1_ctrl_mode_t 正交——前者管电流环怎么跑，后者管 iq_ref 来源。
 * 默认 DISABLED：iq_ref 由 startup / 标定 / API 直接写入，外环不做任何事。
 */
typedef enum {
    M1_OUTER_DISABLED = 0,   /**< 外环关：startup 或标定或 API 直接写 iq_ref */
    M1_OUTER_TORQUE,         /**< 力矩模式：iq_ref = iq_cmd（跳过速度 PI） */
    M1_OUTER_SPEED,          /**< 速度模式：速度 PI → iq_ref */
    M1_OUTER_POSITION,       /**< 位置 P → ω_ref → 速度 PI → iq_ref（多圈 θ_mech） */
} m1_outer_mode_t;

typedef struct {
    uint8_t pole_pairs;
    float uq_open;
    float ud_open;
    m1_ctrl_mode_t mode;
    float id_ref;
    float iq_ref;
    foc_pi_t pi_id;
    foc_pi_t pi_iq;
    float ud_pi;
    float uq_pi;

    /* --- 外环（DISABLED / TORQUE / SPEED / POSITION）--- */
    m1_outer_mode_t outer_mode;
    float omega_ref;         /**< 速度指令 [rpm mech] */
    float theta_ref_rad;     /**< 位置指令 [rad mech，unwrap 连续角] */
    float iq_cmd;            /**< 力矩模式直接指令 [A] */
    foc_pi_t pi_speed;       /**< 速度 PI（2 kHz，输出为 iq_ref） */
} motor_context_t;

#ifdef __cplusplus
}
#endif

#endif
