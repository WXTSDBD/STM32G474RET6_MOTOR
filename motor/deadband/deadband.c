/**
 * @file deadband.c
 * @date 2026-10-06
 * @brief 逆变器死区补偿：仅固定符号法（V_comp / 占空比）。

 *
 * 建表 LUT / geo / baked 已下线。apply_duty 只允许从电流环节拍、写出 PWM 之前调用。
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
 */

#include "deadband.h"

#include <stddef.h>

#include "motor_params_m1.h"

static m1_deadband_cfg_t s_cfg;

static float deadband_phase_sign(float i, float i_zero)
{
    if (M1_DEADBAND_I_ZERO_DISABLE) {
        (void)i_zero;
        if (i > 0.0f) {
            return 1.0f;
        }
        if (i < 0.0f) {
            return -1.0f;
        }
        return 0.0f;
    }
    if (i > i_zero) {
        return 1.0f;
    }
    if (i < -i_zero) {
        return -1.0f;
    }
    return 0.0f;
}

static void deadband_clamp_duty(float *d)
{
    if (*d < 0.0f) {
        *d = 0.0f;
    } else if (*d > 1.0f) {
        *d = 1.0f;
    }
}

/**
 * @brief 按电机参数装默认固定补偿（上电默认 OFF，由 service 切 FIXED）。
 */
void deadband_init(void)
{
    s_cfg.mode = M1_DEADBAND_MODE_OFF;
    s_cfg.v_comp_v = M1_DEADBAND_V_COMP_V;
    s_cfg.i_zero_a = M1_DEADBAND_I_ZERO_A;
    s_cfg.lut_amps = NULL;
    s_cfg.lut_vals = NULL;
    s_cfg.lut_len = 0u;
    s_cfg.lut_triplet = 0u;
    s_cfg.lut_amps_ph[0] = NULL;
    s_cfg.lut_amps_ph[1] = NULL;
    s_cfg.lut_amps_ph[2] = NULL;
    s_cfg.lut_vals_ph[0] = NULL;
    s_cfg.lut_vals_ph[1] = NULL;
    s_cfg.lut_vals_ph[2] = NULL;
}

void deadband_set_lut_runtime_scale(float scale)
{
    (void)scale;
}

float deadband_get_lut_runtime_scale(void)
{
    return 1.0f;
}

/**
 * @brief 切补偿模式。LUT 枚举保留兼容，运行时按 FIXED 处理。
 */
void deadband_set_mode(m1_deadband_mode_t mode)
{
    if (mode == M1_DEADBAND_MODE_LUT) {
        s_cfg.mode = M1_DEADBAND_MODE_FIXED;
        return;
    }
    s_cfg.mode = mode;
}

m1_deadband_mode_t deadband_get_mode(void)
{
    return s_cfg.mode;
}

const m1_deadband_cfg_t *deadband_get_cfg(void)
{
    return &s_cfg;
}

/**
 * @brief 建表已下线：忽略注册。
 */
void deadband_set_lut(const float *amps, const float *vals, uint8_t len)
{
    (void)amps;
    (void)vals;
    (void)len;
}

void deadband_set_phase_luts(const float *amps_a, const float *vals_a,
                             const float *amps_b, const float *vals_b,
                             const float *amps_c, const float *vals_c,
                             uint8_t len)
{
    (void)amps_a;
    (void)vals_a;
    (void)amps_b;
    (void)vals_b;
    (void)amps_c;
    (void)vals_c;
    (void)len;
}

void deadband_set_lut_domain(uint8_t is_d_domain)
{
    (void)is_d_domain;
}

void deadband_set_runtime_apply_ud(uint8_t apply_ud)
{
    (void)apply_ud;
}

void deadband_set_geo_dlut(const float *amps, const float *vals, uint8_t len)
{
    (void)amps;
    (void)vals;
    (void)len;
}

void deadband_set_cluster_luts(const float *amps_a, const float *vals_a,
                               const float *amps_b, const float *vals_b,
                               uint8_t len)
{
    (void)amps_a;
    (void)vals_a;
    (void)amps_b;
    (void)vals_b;
    (void)len;
}

/**
 * @brief 单相补偿电压，单位 V，带符号。OFF 返回 0；否则用固定幅值。
 */
float deadband_comp_v(float i_a)
{
    return deadband_comp_v_ph(0u, i_a);
}

float deadband_comp_v_ph(uint8_t phase, float i_a)
{
    float sign_i;

    (void)phase;
    if (s_cfg.mode == M1_DEADBAND_MODE_OFF) {
        return 0.0f;
    }

    sign_i = deadband_phase_sign(i_a, s_cfg.i_zero_a);
    if (sign_i == 0.0f) {
        return 0.0f;
    }
    return s_cfg.v_comp_v * sign_i;
}

/**
 * @brief d 轴 LUT 路径已下线，恒返回 0。
 */
float deadband_ud_comp_v(float id_a)
{
    (void)id_a;
    return 0.0f;
}

/**
 * @brief 在三相占空比上叠加固定死区补偿并截到 [0,1]。
 */
void deadband_apply_duty(float ia, float ib, float ic,
                         float theta_el, float id_dq, float iq_dq,
                         float *duty_a, float *duty_b, float *duty_c)
{
    float duty_comp;
    float da;
    float db;
    float dc;

    (void)theta_el;
    (void)id_dq;
    (void)iq_dq;

    if (s_cfg.mode == M1_DEADBAND_MODE_OFF) {
        return;
    }

    duty_comp = M1_DEADBAND_DUTY_COMP;
    da = duty_comp * deadband_phase_sign(ia, s_cfg.i_zero_a);
    db = duty_comp * deadband_phase_sign(ib, s_cfg.i_zero_a);
    dc = duty_comp * deadband_phase_sign(ic, s_cfg.i_zero_a);

    *duty_a += da;
    *duty_b += db;
    *duty_c += dc;

    deadband_clamp_duty(duty_a);
    deadband_clamp_duty(duty_b);
    deadband_clamp_duty(duty_c);
}
