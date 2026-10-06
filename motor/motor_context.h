/**
 * @file motor_context.h
 * @date 2026-10-06
 * @brief M1 控制上下文：电流模式、外环模式、PI 影子和指令。
 *
 * 电流环和外环都改这一份。不要在别的文件再复制一份 id_ref / iq_ref。
 *
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
 */

#ifndef MOTOR_CONTEXT_H
#define MOTOR_CONTEXT_H

#include <stdint.h>

#include "foc_pi.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * 电流环怎么跑。与外环模式正交。
 * OPEN_LOOP：开环电压。OBSERVE_ONLY：采样但不出力。CURRENT_LOOP：Id/Iq PI。
 */
typedef enum {
    /** 开环电压，不跑电流 PI。 */
    M1_CTRL_OPEN_LOOP = 0,
    /** 采样和观测，PWM 不出力。 */
    M1_CTRL_OBSERVE_ONLY,
    /** 电流闭环。 */
    M1_CTRL_CURRENT_LOOP,
} m1_ctrl_mode_t;

/** 上电默认电流模式。默认闭环；profile 可覆盖。 */
#ifndef M1_CTRL_MODE_DEFAULT
#define M1_CTRL_MODE_DEFAULT M1_CTRL_CURRENT_LOOP
#endif

/**
 * 谁产生 iq_ref。DISABLED 时由启动、标定或 API 直接写。
 */
typedef enum {
    /** 外环关，iq_ref 由别处写。 */
    M1_OUTER_DISABLED = 0,
    /** 力矩：iq_ref = iq_cmd，不跑速度 PI。 */
    M1_OUTER_TORQUE,
    /** 速度 PI → iq_ref。 */
    M1_OUTER_SPEED,
    /** 位置 P → ω_ref → 速度 PI → iq_ref。θ 是多圈机械角。 */
    M1_OUTER_POSITION,
} m1_outer_mode_t;

typedef struct {
    /** 极对数。 */
    uint8_t pole_pairs;
    /** 开环 Uq，单位 V。 */
    float uq_open;
    /** 开环 Ud，单位 V。 */
    float ud_open;
    /** 电流环模式。 */
    m1_ctrl_mode_t mode;
    /** d 轴电流指令，单位 A。 */
    float id_ref;
    /** q 轴电流指令，单位 A。外环或启动写入。 */
    float iq_ref;
    /** Id PI。 */
    foc_pi_t pi_id;
    /** Iq PI。 */
    foc_pi_t pi_iq;
    /** Id PI 输出 Ud，单位 V。 */
    float ud_pi;
    /** Iq PI 输出 Uq，单位 V。 */
    float uq_pi;

    /** 外环模式。 */
    m1_outer_mode_t outer_mode;
    /** 速度指令，单位机械 rpm。 */
    float omega_ref;
    /** 位置指令，单位机械 rad，unwrap 连续角。 */
    float theta_ref_rad;
    /** 力矩模式的 Iq 指令，单位 A。 */
    float iq_cmd;
    /** 速度 PI。输出为 iq_ref。 */
    foc_pi_t pi_speed;
} motor_context_t;

#ifdef __cplusplus
}
#endif

#endif
