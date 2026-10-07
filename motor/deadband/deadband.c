/**
 * @file deadband.c
 * @date 2026-10-06
 * @brief 运行时死区补偿：FIXED、LUT、Ud 注入和三相占空比。

 *
 * 节拍限制见 deadband.h 文件头。
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
 */

#include "deadband.h"

#include <math.h>
#include <stddef.h>

#include "deadband_geo.h"
#include "motor_cfg.h"
#include "motor_params_m1.h"

#ifndef M1_ID_CAL_D_TO_PHASE_COS
#define M1_ID_CAL_D_TO_PHASE_COS     0.8660254037844386f
#endif

static m1_deadband_cfg_t s_cfg;
static uint8_t s_runtime_apply_ud;
static uint8_t s_lut_domain_d;
static const float *s_geo_dlut_amps;
static const float *s_geo_dlut_vals;
static uint8_t s_geo_dlut_len;
static const float *s_cluster_amps[2];
static const float *s_cluster_vals[2];
static uint8_t s_cluster_lut_len;
static uint8_t s_two_cluster;
static uint8_t s_runtime_cluster;
static float s_lut_runtime_scale;

static const float *deadband_lut_table_amps(uint8_t phase)
{
#if M1_DEADBAND_GEO_TWO_CLUSTER_ENABLE
    if (s_two_cluster != 0u && s_cluster_lut_len >= 2u &&
        s_runtime_cluster < 2u &&
        s_cluster_amps[s_runtime_cluster] != NULL) {
        return s_cluster_amps[s_runtime_cluster];
    }
#endif
    if (s_cfg.lut_triplet != 0u && phase < 3u &&
        s_cfg.lut_amps_ph[phase] != NULL) {
        return s_cfg.lut_amps_ph[phase];
    }
    return s_cfg.lut_amps;
}

static const float *deadband_lut_table_vals(uint8_t phase)
{
#if M1_DEADBAND_GEO_TWO_CLUSTER_ENABLE
    if (s_two_cluster != 0u && s_cluster_lut_len >= 2u &&
        s_runtime_cluster < 2u &&
        s_cluster_vals[s_runtime_cluster] != NULL) {
        return s_cluster_vals[s_runtime_cluster];
    }
#endif
    if (s_cfg.lut_triplet != 0u && phase < 3u &&
        s_cfg.lut_vals_ph[phase] != NULL) {
        return s_cfg.lut_vals_ph[phase];
    }
    return s_cfg.lut_vals;
}

#if M1_DEADBAND_LUT_LOW_FLAT_ENABLE
static float s_lut_flat_amp_lo;
static float s_lut_flat_amp_hi;
static float s_lut_flat_v_lo;
static float s_lut_flat_v_hi;
#endif

static float deadband_phase_sign(float i, float i_zero)
{
#if M1_DEADBAND_I_ZERO_DISABLE
    (void)i_zero;
    if (i > 0.0f) {
        return 1.0f;
    }
    if (i < 0.0f) {
        return -1.0f;
    }
    return 0.0f;
#else
    if (i > i_zero) {
        return 1.0f;
    }
    if (i < -i_zero) {
        return -1.0f;
    }
    return 0.0f;
#endif
}

static float deadband_lut_interp_segment_ph(uint8_t phase, float i_abs)
{
    const float *amps = deadband_lut_table_amps(phase);
    const float *vals = deadband_lut_table_vals(phase);
    uint8_t n = s_cfg.lut_len;

#if M1_DEADBAND_GEO_TWO_CLUSTER_ENABLE
    if (s_two_cluster != 0u && s_cluster_lut_len >= 2u &&
        s_runtime_cluster < 2u &&
        s_cluster_amps[s_runtime_cluster] != NULL) {
        n = s_cluster_lut_len;
    }
#endif
    uint8_t i;

    if (amps == NULL || vals == NULL || n < 2u) {
        return s_cfg.v_comp_v;
    }

    if (i_abs <= amps[0]) {
        return vals[0];
    }
    if (i_abs >= amps[n - 1u]) {
        return vals[n - 1u];
    }

    for (i = 0u; i < (n - 1u); i++) {
        float a0 = amps[i];
        float a1 = amps[i + 1u];

        if (i_abs <= a1) {
            float t = (i_abs - a0) / (a1 - a0);
            return vals[i] + t * (vals[i + 1u] - vals[i]);
        }
    }

    return vals[n - 1u];
}

static float deadband_lut_interp_segment(float i_abs)
{
    return deadband_lut_interp_segment_ph(0u, i_abs);
}

static float deadband_geo_dlut_interp_v(float i_abs)
{
    const float *amps = s_geo_dlut_amps;
    const float *vals = s_geo_dlut_vals;
    uint8_t n = s_geo_dlut_len;
    uint8_t i;

    if (amps == NULL || vals == NULL || n < 2u) {
        return s_cfg.v_comp_v;
    }
    if (i_abs <= amps[0]) {
        return vals[0];
    }
    if (i_abs >= amps[n - 1u]) {
        return vals[n - 1u];
    }
    for (i = 0u; i < (n - 1u); i++) {
        float a0 = amps[i];
        float a1 = amps[i + 1u];

        if (i_abs <= a1) {
            float t = (i_abs - a0) / (a1 - a0);
            return vals[i] + t * (vals[i + 1u] - vals[i]);
        }
    }
    return vals[n - 1u];
}

static void deadband_clamp_duty(float *d)
{
    if (*d < 0.0f) {
        *d = 0.0f;
    } else if (*d > 1.0f) {
        *d = 1.0f;
    }
}

#if M1_DEADBAND_RUNTIME_GEO_ENABLE

static void deadband_apply_duty_geo(float theta_el, float id_dq, float iq_dq,
                                    float ia, float ib, float ic,
                                    float *duty_a, float *duty_b, float *duty_c)
{
    float i_mag;
    float ud_mag;
    float ua;
    float ub;
    float uc;
    float da;
    float db;
    float dc;
    float sign_a;
    float sign_b;
    float sign_c;

    i_mag = sqrtf(id_dq * id_dq + iq_dq * iq_dq);
#if M1_DEADBAND_LUT_APPLY_MIN_ENABLE
    if (i_mag < M1_DEADBAND_LUT_APPLY_MIN_A) {
        return;
    }
#endif
    ud_mag = deadband_geo_dlut_interp_v(i_mag);
    deadband_geo_ud_to_abc(ud_mag, theta_el, &ua, &ub, &uc);
    deadband_geo_remove_u0(&ua, &ub, &uc);

    sign_a = deadband_phase_sign(ia, s_cfg.i_zero_a);
    sign_b = deadband_phase_sign(ib, s_cfg.i_zero_a);
    sign_c = deadband_phase_sign(ic, s_cfg.i_zero_a);

    da = (sign_a == 0.0f) ? 0.0f : (sign_a * fabsf(ua) / g_m1_motor_cfg.vbus_v);
    db = (sign_b == 0.0f) ? 0.0f : (sign_b * fabsf(ub) / g_m1_motor_cfg.vbus_v);
    dc = (sign_c == 0.0f) ? 0.0f : (sign_c * fabsf(uc) / g_m1_motor_cfg.vbus_v);

#if M1_DEADBAND_LUT_ZERO_SEQ_ENABLE
    {
        float u0 = (da + db + dc) / 3.0f;

        da -= u0;
        db -= u0;
        dc -= u0;
    }
#endif

    *duty_a += da;
    *duty_b += db;
    *duty_c += dc;

    deadband_clamp_duty(duty_a);
    deadband_clamp_duty(duty_b);
    deadband_clamp_duty(duty_c);
}

#endif /* M1_DEADBAND_RUNTIME_GEO_ENABLE */

#if M1_DEADBAND_LUT_LOW_FLAT_ENABLE

static void deadband_lut_update_flat_params(void)
{
    if (s_cfg.lut_amps == NULL || s_cfg.lut_vals == NULL || s_cfg.lut_len < 2u) {
        return;
    }

    s_lut_flat_amp_lo = s_cfg.lut_amps[0];
    if (s_lut_domain_d) {
        s_lut_flat_amp_hi = M1_DEADBAND_LUT_LOW_FLAT_ID_A;
    } else {
        s_lut_flat_amp_hi = M1_DEADBAND_LUT_LOW_FLAT_ID_A * M1_ID_CAL_D_TO_PHASE_COS;
    }
    s_lut_flat_v_lo = s_cfg.lut_vals[0];
    s_lut_flat_v_hi = deadband_lut_interp_segment(s_lut_flat_amp_hi);
}

#endif /* M1_DEADBAND_LUT_LOW_FLAT_ENABLE */

static float deadband_lut_lookup_v_ph(uint8_t phase, float i_abs)
{
    float v;

#if M1_DEADBAND_LUT_LOW_FLAT_ENABLE
    if (!s_cfg.lut_triplet && s_cfg.lut_amps != NULL && s_cfg.lut_vals != NULL &&
        s_cfg.lut_len >= 2u &&
        s_lut_flat_amp_hi > s_lut_flat_amp_lo && i_abs <= s_lut_flat_amp_hi) {
        float t = (i_abs - s_lut_flat_amp_lo) / (s_lut_flat_amp_hi - s_lut_flat_amp_lo);
        v = s_lut_flat_v_lo + t * (s_lut_flat_v_hi - s_lut_flat_v_lo);
    } else
#endif
    {
        v = deadband_lut_interp_segment_ph(phase, i_abs);
    }

    if (s_lut_domain_d == 0u) {
        v *= s_lut_runtime_scale;
    }
    return v;
}

static float deadband_lut_lookup_v(float i_abs)
{
    return deadband_lut_lookup_v_ph(0u, i_abs);
}

static float deadband_comp_mag_v_ph(uint8_t phase, float i_abs)
{
    if (s_cfg.mode == M1_DEADBAND_MODE_LUT) {
#if M1_DEADBAND_LUT_APPLY_MIN_ENABLE
        if (i_abs < M1_DEADBAND_LUT_APPLY_MIN_A) {
            return 0.0f;
        }
#endif
        return deadband_lut_lookup_v_ph(phase, i_abs);
    }
    return s_cfg.v_comp_v;
}

static float deadband_comp_mag_v(float i_abs)
{
    return deadband_comp_mag_v_ph(0u, i_abs);
}

/**
 * @brief 按电机参数装默认固定补偿。
 */
void deadband_init(void)
{
    /* 上电默认不补偿；标定/探路/ident 会话内显式 set_mode */
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
    s_runtime_apply_ud = 0u;
    s_lut_domain_d = 0u;
    s_geo_dlut_amps = NULL;
    s_geo_dlut_vals = NULL;
    s_geo_dlut_len = 0u;
    s_cluster_amps[0] = NULL;
    s_cluster_amps[1] = NULL;
    s_cluster_vals[0] = NULL;
    s_cluster_vals[1] = NULL;
    s_cluster_lut_len = 0u;
    s_two_cluster = 0u;
    s_runtime_cluster = 0u;
    s_lut_runtime_scale = M1_DEADBAND_LUT_RUNTIME_SCALE;
}

void deadband_set_lut_runtime_scale(float scale)
{
    if (scale >= 0.05f && scale <= 1.5f) {
        s_lut_runtime_scale = scale;
    }
}

float deadband_get_lut_runtime_scale(void)
{
    return s_lut_runtime_scale;
}

/**
 * @brief 切补偿模式。
 */
void deadband_set_mode(m1_deadband_mode_t mode)
{
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
 * @brief 注册单表。len 须至少 2。
 */
void deadband_set_lut(const float *amps, const float *vals, uint8_t len)
{
    if (amps == NULL || vals == NULL || len < 2u || len > M1_DEADBAND_LUT_MAX) {
        return;
    }

    s_cfg.lut_amps = amps;
    s_cfg.lut_vals = vals;
    s_cfg.lut_len = len;
    s_cfg.lut_triplet = 0u;
    s_cfg.lut_amps_ph[0] = amps;
    s_cfg.lut_amps_ph[1] = amps;
    s_cfg.lut_amps_ph[2] = amps;
    s_cfg.lut_vals_ph[0] = vals;
    s_cfg.lut_vals_ph[1] = vals;
    s_cfg.lut_vals_ph[2] = vals;
    s_cfg.mode = M1_DEADBAND_MODE_LUT;
#if M1_DEADBAND_LUT_LOW_FLAT_ENABLE
    deadband_lut_update_flat_params();
#endif
}

void deadband_set_phase_luts(const float *amps_a, const float *vals_a,
                             const float *amps_b, const float *vals_b,
                             const float *amps_c, const float *vals_c,
                             uint8_t len)
{
    if (amps_a == NULL || vals_a == NULL ||
        amps_b == NULL || vals_b == NULL ||
        amps_c == NULL || vals_c == NULL ||
        len < 2u || len > M1_DEADBAND_LUT_MAX) {
        return;
    }

    s_cfg.lut_amps = amps_a;
    s_cfg.lut_vals = vals_a;
    s_cfg.lut_len = len;
    s_cfg.lut_triplet = 1u;
    s_cfg.lut_amps_ph[0] = amps_a;
    s_cfg.lut_amps_ph[1] = amps_b;
    s_cfg.lut_amps_ph[2] = amps_c;
    s_cfg.lut_vals_ph[0] = vals_a;
    s_cfg.lut_vals_ph[1] = vals_b;
    s_cfg.lut_vals_ph[2] = vals_c;
    s_cfg.mode = M1_DEADBAND_MODE_LUT;
}

void deadband_set_lut_domain(uint8_t is_d_domain)
{
    s_lut_domain_d = is_d_domain ? 1u : 0u;
#if M1_DEADBAND_LUT_LOW_FLAT_ENABLE
    deadband_lut_update_flat_params();
#endif
}

void deadband_set_runtime_apply_ud(uint8_t apply_ud)
{
    s_runtime_apply_ud = apply_ud ? 1u : 0u;
}

void deadband_set_geo_dlut(const float *amps, const float *vals, uint8_t len)
{
    if (amps == NULL || vals == NULL || len < 2u || len > M1_DEADBAND_LUT_MAX) {
        s_geo_dlut_amps = NULL;
        s_geo_dlut_vals = NULL;
        s_geo_dlut_len = 0u;
        return;
    }

    s_geo_dlut_amps = amps;
    s_geo_dlut_vals = vals;
    s_geo_dlut_len = len;
}

void deadband_set_cluster_luts(const float *amps_a, const float *vals_a,
                               const float *amps_b, const float *vals_b,
                               uint8_t len)
{
    if (amps_a == NULL || vals_a == NULL ||
        amps_b == NULL || vals_b == NULL ||
        len < 2u || len > M1_DEADBAND_LUT_MAX) {
        s_two_cluster = 0u;
        s_cluster_lut_len = 0u;
        return;
    }

    s_cluster_amps[0] = amps_a;
    s_cluster_vals[0] = vals_a;
    s_cluster_amps[1] = amps_b;
    s_cluster_vals[1] = vals_b;
    s_cluster_lut_len = len;
    s_two_cluster = 1u;
}

/**
 * @brief 单相补偿电压，单位 V，带符号。OFF 返回 0。
 */
float deadband_comp_v(float i_a)
{
    return deadband_comp_v_ph(0u, i_a);
}

float deadband_comp_v_ph(uint8_t phase, float i_a)
{
    float i_abs;
    float mag_v;
    float sign_i;

    if (s_cfg.mode == M1_DEADBAND_MODE_OFF) {
        return 0.0f;
    }

    if (phase > 2u) {
        phase = 0u;
    }

    sign_i = deadband_phase_sign(i_a, s_cfg.i_zero_a);
    if (sign_i == 0.0f) {
        return 0.0f;
    }

    i_abs = (i_a >= 0.0f) ? i_a : -i_a;
    mag_v = deadband_comp_mag_v_ph(phase, i_abs);
    return mag_v * sign_i;
}

/**
 * @brief d 轴补偿电压，单位 V。仅 LUT 且 Ud 路径打开时用。
 */
float deadband_ud_comp_v(float id_a)
{
    float id_abs;
    float mag_v;

#if !M1_DEADBAND_LUT_APPLY_UD
    (void)id_a;
    return 0.0f;
#else
    if (s_cfg.mode != M1_DEADBAND_MODE_LUT || !s_runtime_apply_ud) {
        return 0.0f;
    }

    id_abs = (id_a >= 0.0f) ? id_a : -id_a;
#if M1_DEADBAND_LUT_APPLY_MIN_ENABLE
    if (id_abs < M1_DEADBAND_LUT_APPLY_MIN_A) {
        return 0.0f;
    }
#endif
#if !M1_DEADBAND_I_ZERO_DISABLE
    if (id_abs < s_cfg.i_zero_a) {
        return 0.0f;
    }
#endif

    mag_v = deadband_lut_lookup_v(id_abs);
    return (id_a >= 0.0f) ? mag_v : -mag_v;
#endif
}

/**
 * @brief 在三相占空比上叠加补偿并截到 [0,1]。
 */
void deadband_apply_duty(float ia, float ib, float ic,
                         float theta_el, float id_dq, float iq_dq,
                         float *duty_a, float *duty_b, float *duty_c)
{
    float duty_comp;
    float da;
    float db;
    float dc;

    if (s_cfg.mode == M1_DEADBAND_MODE_OFF) {
        return;
    }

    if (s_cfg.mode == M1_DEADBAND_MODE_LUT && s_runtime_apply_ud) {
        return;
    }

#if M1_DEADBAND_RUNTIME_GEO_ENABLE
    if (s_cfg.mode == M1_DEADBAND_MODE_LUT && s_geo_dlut_len >= 2u) {
        deadband_apply_duty_geo(theta_el, id_dq, iq_dq, ia, ib, ic,
                                duty_a, duty_b, duty_c);
        return;
    }
#endif

#if M1_DEADBAND_GEO_TWO_CLUSTER_ENABLE
    if (s_two_cluster != 0u && s_cluster_lut_len >= 2u) {
        s_runtime_cluster = deadband_geo_theta_cluster_idx(theta_el);
    }
#endif

    duty_comp = M1_DEADBAND_DUTY_COMP;

    if (s_cfg.mode == M1_DEADBAND_MODE_FIXED) {
        da = duty_comp * deadband_phase_sign(ia, s_cfg.i_zero_a);
        db = duty_comp * deadband_phase_sign(ib, s_cfg.i_zero_a);
        dc = duty_comp * deadband_phase_sign(ic, s_cfg.i_zero_a);
    } else {
        da = deadband_comp_v_ph(0u, ia) / g_m1_motor_cfg.vbus_v;
        db = deadband_comp_v_ph(1u, ib) / g_m1_motor_cfg.vbus_v;
        dc = deadband_comp_v_ph(2u, ic) / g_m1_motor_cfg.vbus_v;
    }

#if M1_DEADBAND_LUT_ZERO_SEQ_ENABLE
    {
        float u0 = (da + db + dc) / 3.0f;

        da -= u0;
        db -= u0;
        dc -= u0;
    }
#endif

    *duty_a += da;
    *duty_b += db;
    *duty_c += dc;

    deadband_clamp_duty(duty_a);
    deadband_clamp_duty(duty_b);
    deadband_clamp_duty(duty_c);
}
