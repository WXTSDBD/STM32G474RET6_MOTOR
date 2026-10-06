/**
 * @file ld_lq_ident.c
 * @date 2026-10-06
 * @brief VASI Ld/Lq 辨识实现。

 *
 * 节拍限制见 ld_lq_ident.h 文件头。
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
 */

#include "ld_lq_ident.h"

#if M1_LD_LQ_IDENT_ENABLE

#include <math.h>
#include <string.h>

#include "dbg_monitor.h"
#include "motor_params_m1.h"
#if M1_LD_LQ_IDENT_INJECT_LUT_ENABLE
#include "deadband_service.h"
#endif

typedef enum {
    LD_LQ_SUB_SETTLE = 0,
    LD_LQ_SUB_INJ_LD,
    LD_LQ_SUB_INJ_LQ,
    LD_LQ_SUB_DONE,
} ld_lq_sub_state_t;

typedef enum {
    LD_INJ_HALF_POS = 0,
    LD_INJ_HALF_NEG,
} ld_inj_half_t;

typedef enum {
    LD_LQ_INJ_TIER_COARSE = 0, /* 500 Hz */
    LD_LQ_INJ_TIER_FINE = 1,   /* M1_LD_LQ_IDENT_F_FINE_HZ */
#if M1_LD_LQ_IDENT_F2_ENABLE
    LD_LQ_INJ_TIER_F2 = 2, /* M1_LD_LQ_IDENT_F_F2_HZ */
#endif
} ld_lq_inj_tier_t;

static const float s_id_bias_table[M1_LD_LQ_ID_BIAS_N] = {
#if M1_LD_LQ_IDENT_BODE_BIAS_GRID_ENABLE
    0.0f,
#elif M1_LD_LQ_IDENT_FINE_GRID_ENABLE
    0.5f, 0.75f, 1.0f,
#else
    0.5f, 1.0f, 1.5f,
#endif
};
static const float s_iq_bias_table[M1_LD_LQ_IQ_BIAS_N] = {
#if M1_LD_LQ_IDENT_BODE_BIAS_GRID_ENABLE
    0.25f, 1.25f,
#elif M1_LD_LQ_IDENT_FINE_GRID_ENABLE
    0.0f, 0.25f, 0.5f, 0.75f, 1.0f,
#else
    0.0f, 0.5f, 1.0f,
#endif
};

static ld_lq_sub_state_t s_sub;
static ld_inj_half_t s_inj_half;
static uint8_t s_grid_idx;
static uint8_t s_amp_idx;
static uint8_t s_cycle_idx;
static uint8_t s_done;
static uint8_t s_ok;
static uint8_t s_inj_axis; /* 0=d, 1=q */
static ld_lq_inj_tier_t s_inj_tier;
static float s_id_bias;
static float s_iq_bias;
static float s_id_ref_cmd;
static float s_iq_ref_cmd;
static float s_id_ref_start;
static float s_iq_ref_start;
static float s_rs_ohm;
static float s_v_inj;
static float s_u_inj_d;
static float s_u_inj_q;
static float s_f_hz;

static float s_theta_ref_el;
static float s_theta_drift_max_el;

static uint32_t s_settle_tick;
static uint32_t s_settle_elapsed;
static uint32_t s_half_tick;
static uint32_t s_half_period_ticks;

static float s_i_start;
static float s_sum_psi;

static float s_amp_l_buf[M1_LD_LQ_IDENT_AMP_STEPS * 2u];
static uint8_t s_amp_l_valid_n;
static float s_amp_cycle_l_sum;
static uint8_t s_amp_cycle_l_n;

static float s_inj_delta_i_pos;
static float s_inj_psi_pos;
static uint8_t s_inj_have_pos;
static float s_half_u_ac_sum;
static float s_u_ac_avg_pos;

static float s_ud_pi_cache;
static float s_uq_pi_cache;
#if M1_LD_LQ_IDENT_OPEN_LOOP_ENABLE
static float s_u_bias_d;
static float s_u_bias_q;
#endif
/** coarse 500 Hz 段中位 L；fine/f2 各频中位 L 分存 */
static float s_coarse_l_h;
static uint8_t s_coarse_l_valid;
/** 1=SETTLE 内线性 ramp（仅 leg 入口 G0）；0=格点间阶跃切偏置 */
static uint8_t s_bias_ramp_en;

static ld_lq_ident_result_t s_result;

#if M1_LD_LQ_MULTI_ANGLE_ENABLE
static ld_lq_ident_result_t s_result_leg[M1_LD_LQ_IDENT_ANGLE_COUNT];
static float s_theta_target_rad[M1_LD_LQ_IDENT_ANGLE_COUNT];
static uint8_t s_angle_leg;
#endif

#ifndef LD_LQ_PI
#define LD_LQ_PI 3.14159265358979323846f
#endif

static uint32_t ld_lq_ticks_from_s(float s)
{
    if (s <= 0.0f) {
        return 0u;
    }
    return (uint32_t)(s / M1_CTRL_TS_S + 0.5f);
}

static float ld_lq_wrap_pi(float a)
{
    while (a > LD_LQ_PI) {
        a -= 2.0f * LD_LQ_PI;
    }
    while (a < -LD_LQ_PI) {
        a += 2.0f * LD_LQ_PI;
    }
    return a;
}

static float ld_lq_theta_drift_mech_deg(float theta_el)
{
    const float d_el = ld_lq_wrap_pi(theta_el - s_theta_ref_el);

    return (d_el * 180.0f / LD_LQ_PI) / (float)M1_POLE_PAIRS;
}

static float ld_lq_f_hz_from_tier(ld_lq_inj_tier_t tier)
{
    switch (tier) {
    case LD_LQ_INJ_TIER_COARSE:
        return M1_LD_LQ_IDENT_F_COARSE_HZ;
    case LD_LQ_INJ_TIER_FINE:
        return M1_LD_LQ_IDENT_F_FINE_HZ;
#if M1_LD_LQ_IDENT_F2_ENABLE
    case LD_LQ_INJ_TIER_F2:
        return M1_LD_LQ_IDENT_F_F2_HZ;
#endif
    default:
        return M1_LD_LQ_IDENT_F_FINE_HZ;
    }
}

static uint32_t ld_lq_proc_tier_band(ld_lq_inj_tier_t tier)
{
    /* proc_code 频带：1→500 Hz，0→1 kHz，2→2 kHz（兼容旧 coarse*5 编码） */
    if (tier == LD_LQ_INJ_TIER_COARSE) {
        return 1u;
    }
#if M1_LD_LQ_IDENT_F2_ENABLE
    if (tier == LD_LQ_INJ_TIER_F2) {
        return 2u;
    }
#endif
    return 0u;
}

static ld_lq_inj_tier_t ld_lq_inj_tier_first(void)
{
#if M1_LD_LQ_IDENT_FINE_ONLY
    return LD_LQ_INJ_TIER_FINE;
#else
    return LD_LQ_INJ_TIER_COARSE;
#endif
}

static uint32_t ld_lq_half_period_ticks_from_f(float f_hz)
{
    const float half_s = 0.5f / f_hz;

    if (half_s <= 0.0f) {
        return 2u;
    }
    {
        const uint32_t n = (uint32_t)(half_s / M1_CTRL_TS_S + 0.5f);

        return (n < 2u) ? 2u : n;
    }
}

static void ld_lq_update_half_period(void)
{
    s_half_period_ticks = ld_lq_half_period_ticks_from_f(s_f_hz);
}

static void ld_lq_grid_bias_from_idx(uint8_t idx, float *id0, float *iq0)
{
    const uint8_t iq_i = (uint8_t)(idx % M1_LD_LQ_IQ_BIAS_N);
    const uint8_t id_i = (uint8_t)(idx / M1_LD_LQ_IQ_BIAS_N);

    if (id0 != 0) {
        *id0 = s_id_bias_table[id_i];
    }
    if (iq0 != 0) {
        *iq0 = s_iq_bias_table[iq_i];
    }
}

static float ld_lq_inj_i_bias_floor(void)
{
    if (s_inj_axis == 0u) {
        return fabsf(s_id_bias);
    }
    return fabsf(s_iq_bias);
}

static float ld_lq_u_axis_limit(float ud_bias, float uq_bias, uint8_t axis)
{
    const float v_lim = M1_PI_V_LIMIT_V;
    const float orth2 = (axis == 0u) ? (uq_bias * uq_bias) : (ud_bias * ud_bias);
    const float r2 = v_lim * v_lim - orth2;

    if (r2 <= 1e-6f) {
        return 0.0f;
    }
    return sqrtf(r2);
}

/** 保证 bias±v 同幅时仍在电压极限圆内 */
static float ld_lq_symmetric_v_max(float u_bias_axis, float u_axis_limit)
{
    float v_max;

    if (u_axis_limit <= 0.0f) {
        return 0.0f;
    }
    v_max = fminf(u_axis_limit - u_bias_axis, u_bias_axis + u_axis_limit);
    if (v_max < M1_LD_LQ_IDENT_U_SYM_MIN_DV_V) {
        return 0.0f;
    }
    return v_max;
}

static float ld_lq_clamp_v_inj(float v_req, float ud_bias, float uq_bias)
{
    const float v_lim = M1_PI_V_LIMIT_V;
    float v = v_req;
    float u_axis_limit;
    float v_sym;
    float ud;
    float uq;
    float mag;

    if (v <= 0.0f) {
        return 0.0f;
    }

    u_axis_limit = ld_lq_u_axis_limit(ud_bias, uq_bias, s_inj_axis);
    v_sym = ld_lq_symmetric_v_max(
        (s_inj_axis == 0u) ? ud_bias : uq_bias, u_axis_limit);
    v = fminf(v, v_sym);

    if (s_inj_tier != LD_LQ_INJ_TIER_COARSE) {
        v = fminf(v, M1_LD_LQ_IDENT_FINE_V_INJ_MAX_V);
    }

    if (s_inj_axis == 0u) {
        ud = ud_bias + v;
        uq = uq_bias;
        mag = sqrtf(ud * ud + uq * uq);
        if (mag > v_lim && mag > 1e-6f) {
            v *= v_lim / mag;
        }
        ud = ud_bias - v;
        mag = sqrtf(ud * ud + uq * uq);
        if (mag > v_lim && mag > 1e-6f) {
            v *= v_lim / mag;
        }
    } else {
        ud = ud_bias;
        uq = uq_bias + v;
        mag = sqrtf(ud * ud + uq * uq);
        if (mag > v_lim && mag > 1e-6f) {
            v *= v_lim / mag;
        }
        uq = uq_bias - v;
        mag = sqrtf(ud * ud + uq * uq);
        if (mag > v_lim && mag > 1e-6f) {
            v *= v_lim / mag;
        }
    }

    return (v >= M1_LD_LQ_IDENT_U_SYM_MIN_DV_V) ? v : 0.0f;
}

static float ld_lq_v_inj_cap_by_bias(void)
{
    const float i_room = ld_lq_inj_i_bias_floor() - M1_LD_LQ_IDENT_I_RIPPLE_MARGIN_A;

    if (i_room <= 0.0f) {
        return M1_LD_LQ_IDENT_U_INJ_MIN_V;
    }
    return 2.0f * s_f_hz * M1_LD_LQ_IDENT_L_NOM_H * i_room;
}

static float ld_lq_v_inj_from_step(uint8_t step)
{
    float t;
    float u_lo;
    float u_hi;
    float u_at_di;

    if (M1_LD_LQ_IDENT_AMP_STEPS <= 1u) {
        return M1_LD_LQ_IDENT_U_INJ_MIN_V;
    }

    u_at_di = M1_LD_LQ_IDENT_DI_TARGET_A * 2.0f * s_f_hz * M1_LD_LQ_IDENT_L_NOM_H;
    u_lo = fmaxf(M1_LD_LQ_IDENT_U_INJ_MIN_V, u_at_di * 0.80f);
    u_hi = fminf(M1_LD_LQ_IDENT_U_INJ_MAX_V, fmaxf(u_at_di * 1.20f, u_lo + 0.04f));
    u_hi = fminf(u_hi, ld_lq_v_inj_cap_by_bias());

    if (u_hi < u_lo) {
        u_hi = u_lo;
    }

    t = (float)step / (float)(M1_LD_LQ_IDENT_AMP_STEPS - 1u);
    return u_lo + t * (u_hi - u_lo);
}

static float ld_lq_median_f(float *buf, uint8_t n)
{
    float tmp[M1_LD_LQ_IDENT_AMP_STEPS * 2u];
    uint8_t i;
    uint8_t j;
    uint8_t min_i;
    float t;

    if (n == 0u) {
        return 0.0f;
    }
    for (i = 0u; i < n; i++) {
        tmp[i] = buf[i];
    }
    for (i = 0u; i < n; i++) {
        min_i = i;
        for (j = (uint8_t)(i + 1u); j < n; j++) {
            if (tmp[j] < tmp[min_i]) {
                min_i = j;
            }
        }
        t = tmp[i];
        tmp[i] = tmp[min_i];
        tmp[min_i] = t;
    }
    if ((n & 1u) != 0u) {
        return tmp[n / 2u];
    }
    return 0.5f * (tmp[n / 2u - 1u] + tmp[n / 2u]);
}

static void ld_lq_abort(void)
{
    s_sub = LD_LQ_SUB_DONE;
    s_done = 1u;
    s_ok = 0u;
    s_u_inj_d = 0.0f;
    s_u_inj_q = 0.0f;
    s_result.ok = 0u;
    ld_lq_ident_sync_dbg();
}

static uint8_t ld_lq_check_theta_drift(float theta_el)
{
    const float d_el = fabsf(ld_lq_wrap_pi(theta_el - s_theta_ref_el));

    dbg.ld_lq_theta_drift_mech_deg = ld_lq_theta_drift_mech_deg(theta_el);
#if M1_LD_LQ_IDENT_ABORT_ON_THETA_DRIFT
    if (d_el > s_theta_drift_max_el) {
        ld_lq_abort();
        return 1u;
    }
#endif
    (void)d_el;
    return 0u;
}

static void ld_lq_inj_reset_half(void)
{
    s_half_tick = 0u;
    s_sum_psi = 0.0f;
    s_half_u_ac_sum = 0.0f;
}

static void ld_lq_inj_begin_amp(float ud_bias, float uq_bias)
{
    const float v_req = ld_lq_v_inj_from_step(s_amp_idx);

    s_v_inj = ld_lq_clamp_v_inj(v_req, ud_bias, uq_bias);
    s_cycle_idx = 0u;
    s_inj_half = LD_INJ_HALF_POS;
    ld_lq_inj_reset_half();
}

static void ld_lq_inj_set_outputs(void)
{
    const float v = s_v_inj;

    s_u_inj_d = 0.0f;
    s_u_inj_q = 0.0f;
    if (s_inj_axis == 0u) {
        s_u_inj_d = (s_inj_half == LD_INJ_HALF_POS) ? v : -v;
    } else {
        s_u_inj_q = (s_inj_half == LD_INJ_HALF_POS) ? v : -v;
    }
}

static float ld_lq_axis_u_out(float ud_out, float uq_out)
{
    if (s_inj_axis == 0u) {
        return ud_out;
    }
    return uq_out;
}

static float ld_lq_axis_u_bias(void)
{
#if M1_LD_LQ_IDENT_OPEN_LOOP_ENABLE
    if (s_inj_axis == 0u) {
        return s_u_bias_d;
    }
    return s_u_bias_q;
#else
    if (s_inj_axis == 0u) {
        return s_ud_pi_cache;
    }
    return s_uq_pi_cache;
#endif
}

static float ld_lq_axis_u_for_psi(float ud_out, float uq_out)
{
    const float u = ld_lq_axis_u_out(ud_out, uq_out);

#if M1_LD_LQ_IDENT_PSI_USE_U_AC
    return u - ld_lq_axis_u_bias();
#else
    return u;
#endif
}

static uint8_t ld_lq_half_u_ac_sym_ok(float u_ac_pos, float u_ac_neg)
{
    const float a = fabsf(u_ac_pos);
    const float b = fabsf(u_ac_neg);
    float ratio;

    if (a < M1_LD_LQ_IDENT_U_SYM_MIN_DV_V || b < M1_LD_LQ_IDENT_U_SYM_MIN_DV_V) {
        return 0u;
    }
    ratio = fminf(a, b) / fmaxf(a, b);
    return (ratio >= M1_LD_LQ_IDENT_U_SYM_RATIO_MIN) ? 1u : 0u;
}

static float ld_lq_axis_i(float id_fb, float iq_fb)
{
    if (s_inj_axis == 0u) {
        return id_fb;
    }
    return iq_fb;
}

static uint8_t ld_lq_l_valid(float l_h)
{
    return (l_h >= M1_LD_LQ_IDENT_L_MIN_H && l_h < M1_LD_LQ_IDENT_L_MAX_H) ? 1u : 0u;
}

static void ld_lq_inj_reset_cycle_state(void)
{
    s_inj_have_pos = 0u;
    s_inj_delta_i_pos = 0.0f;
    s_inj_psi_pos = 0.0f;
}

static uint8_t ld_lq_try_finish_half(float id_fb, float iq_fb, float ud_out, float uq_out)
{
    const float u = ld_lq_axis_u_for_psi(ud_out, uq_out);
    const float i = ld_lq_axis_i(id_fb, iq_fb);
    float i_end;
    float delta_i;
    float psi_half;
    float l_h;
    float u_ac_avg;

    s_sum_psi += (u - s_rs_ohm * i) * M1_CTRL_TS_S;
    s_half_u_ac_sum += u;
    s_half_tick++;

    if (s_half_tick < s_half_period_ticks) {
        return 0u;
    }

    u_ac_avg = s_half_u_ac_sum / (float)s_half_period_ticks;
    i_end = i;
    delta_i = i_end - s_i_start;
    psi_half = s_sum_psi;

    if (s_inj_half == LD_INJ_HALF_POS) {
        s_inj_delta_i_pos = delta_i;
        s_inj_psi_pos = psi_half;
        s_u_ac_avg_pos = u_ac_avg;
        s_inj_have_pos = 1u;
        s_inj_half = LD_INJ_HALF_NEG;
        ld_lq_inj_reset_half();
        s_i_start = i;
        return 0u;
    }

    if (s_inj_have_pos == 0u) {
        s_inj_half = LD_INJ_HALF_POS;
        ld_lq_inj_reset_half();
        s_i_start = i;
        return 0u;
    }

    {
        const float den = s_inj_delta_i_pos - delta_i;

        if (fabsf(den) >= M1_LD_LQ_IDENT_MIN_DI_A &&
            ld_lq_half_u_ac_sym_ok(s_u_ac_avg_pos, u_ac_avg) != 0u) {
            l_h = (s_inj_psi_pos - psi_half) / den;
            dbg.ld_lq_proc_psi_du_wb = s_inj_psi_pos - psi_half;
            dbg.ld_lq_proc_di_den_a = den;
            if (ld_lq_l_valid(l_h)) {
                dbg.ld_lq_proc_last_l_uH = l_h * 1e6f;
                s_amp_cycle_l_sum += l_h;
                s_amp_cycle_l_n++;
            }
        }
    }

    s_inj_have_pos = 0u;
    s_cycle_idx++;
    if (s_cycle_idx >= M1_LD_LQ_IDENT_CYCLES_PER_AMP) {
        s_cycle_idx = 0u;
        if (s_amp_cycle_l_n > 0u) {
            if (s_amp_l_valid_n < (uint8_t)(sizeof(s_amp_l_buf) / sizeof(s_amp_l_buf[0]))) {
                s_amp_l_buf[s_amp_l_valid_n] = s_amp_cycle_l_sum / (float)s_amp_cycle_l_n;
                s_amp_l_valid_n++;
            }
            s_amp_cycle_l_sum = 0.0f;
            s_amp_cycle_l_n = 0u;
        }
        s_amp_idx++;
        if (s_amp_idx >= M1_LD_LQ_IDENT_AMP_STEPS) {
            return 1u;
        }
        ld_lq_inj_begin_amp(s_ud_pi_cache, s_uq_pi_cache);
    }

    s_inj_half = LD_INJ_HALF_POS;
    ld_lq_inj_reset_half();
    s_i_start = i;
    return 0u;
}

static void ld_lq_store_axis_coarse(uint8_t axis)
{
    if (s_coarse_l_valid == 0u) {
        return;
    }

    s_result.id_bias[s_grid_idx] = s_id_bias;
    s_result.iq_bias[s_grid_idx] = s_iq_bias;
    if (axis == 0u) {
        s_result.ld_h[s_grid_idx] = s_coarse_l_h;
        s_result.ld_valid[s_grid_idx] = ld_lq_l_valid(s_coarse_l_h) ? 1u : 0u;
        if (s_result.ld_valid[s_grid_idx] != 0u) {
            s_result.n_ld_ok++;
        }
    } else {
        s_result.lq_h[s_grid_idx] = s_coarse_l_h;
        s_result.lq_valid[s_grid_idx] = ld_lq_l_valid(s_coarse_l_h) ? 1u : 0u;
        if (s_result.lq_valid[s_grid_idx] != 0u) {
            s_result.n_lq_ok++;
        }
    }
    s_coarse_l_valid = 0u;
}

static void ld_lq_store_axis_median(uint8_t axis, ld_lq_inj_tier_t tier)
{
    float l_h;

    if (s_amp_l_valid_n == 0u) {
        return;
    }
    l_h = ld_lq_median_f(s_amp_l_buf, s_amp_l_valid_n);
    if (tier == LD_LQ_INJ_TIER_FINE) {
        if (axis == 0u) {
            s_result.id_bias[s_grid_idx] = s_id_bias;
            s_result.iq_bias[s_grid_idx] = s_iq_bias;
            s_result.ld_h_fine[s_grid_idx] = l_h;
            s_result.ld_valid_fine[s_grid_idx] = ld_lq_l_valid(l_h) ? 1u : 0u;
            if (s_result.ld_valid_fine[s_grid_idx] != 0u) {
                s_result.n_ld_ok_fine++;
            }
        } else {
            s_result.lq_h_fine[s_grid_idx] = l_h;
            s_result.lq_valid_fine[s_grid_idx] = ld_lq_l_valid(l_h) ? 1u : 0u;
            if (s_result.lq_valid_fine[s_grid_idx] != 0u) {
                s_result.n_lq_ok_fine++;
            }
        }
        return;
    }
#if M1_LD_LQ_IDENT_F2_ENABLE
    if (tier == LD_LQ_INJ_TIER_F2) {
        if (axis == 0u) {
            s_result.ld_h_f2[s_grid_idx] = l_h;
            s_result.ld_valid_f2[s_grid_idx] = ld_lq_l_valid(l_h) ? 1u : 0u;
            if (s_result.ld_valid_f2[s_grid_idx] != 0u) {
                s_result.n_ld_ok_f2++;
            }
        } else {
            s_result.lq_h_f2[s_grid_idx] = l_h;
            s_result.lq_valid_f2[s_grid_idx] = ld_lq_l_valid(l_h) ? 1u : 0u;
            if (s_result.lq_valid_f2[s_grid_idx] != 0u) {
                s_result.n_lq_ok_f2++;
            }
        }
    }
#else
    (void)axis;
    (void)tier;
#endif
}

static void ld_lq_update_result_ok(void)
{
    s_result.ok = 0u;
    s_result.ok_fine = 0u;
#if M1_LD_LQ_IDENT_F2_ENABLE
    s_result.ok_f2 = 0u;
#endif
    if (s_result.n_ld_ok >= M1_LD_LQ_IDENT_MIN_LD_OK &&
        s_result.n_lq_ok >= M1_LD_LQ_IDENT_MIN_LQ_OK) {
        s_result.ok = 1u;
    }
    if (s_result.n_ld_ok_fine >= M1_LD_LQ_IDENT_MIN_LD_OK &&
        s_result.n_lq_ok_fine >= M1_LD_LQ_IDENT_MIN_LQ_OK) {
        s_result.ok_fine = 1u;
    }
#if M1_LD_LQ_IDENT_FINE_ONLY
    if (s_result.ok_fine != 0u) {
        s_result.ok = 1u;
    }
#endif
#if M1_LD_LQ_IDENT_F2_ENABLE
    if (s_result.n_ld_ok_f2 >= M1_LD_LQ_IDENT_MIN_LD_OK &&
        s_result.n_lq_ok_f2 >= M1_LD_LQ_IDENT_MIN_LQ_OK) {
        s_result.ok_f2 = 1u;
    }
#endif
}

static void ld_lq_capture_coarse_l(void)
{
    if (s_amp_l_valid_n == 0u) {
        s_coarse_l_valid = 0u;
        return;
    }
    s_coarse_l_h = ld_lq_median_f(s_amp_l_buf, s_amp_l_valid_n);
    s_coarse_l_valid = 1u;
}

#if M1_LD_LQ_IDENT_INJECT_LUT_ENABLE
static uint8_t ld_lq_inject_active(void);
static void ld_lq_sync_inject_deadband_profile(void);
#endif

static void ld_lq_begin_inject(uint8_t axis, ld_lq_inj_tier_t tier, float ud_bias, float uq_bias)
{
    s_inj_axis = axis;
    s_inj_tier = tier;
    s_f_hz = ld_lq_f_hz_from_tier(tier);
    ld_lq_update_half_period();
    s_amp_idx = 0u;
    s_amp_l_valid_n = 0u;
    if (tier == LD_LQ_INJ_TIER_COARSE) {
        s_coarse_l_valid = 0u;
    }
    s_amp_cycle_l_sum = 0.0f;
    s_amp_cycle_l_n = 0u;
    ld_lq_inj_reset_cycle_state();
    ld_lq_inj_begin_amp(ud_bias, uq_bias);
    ld_lq_inj_reset_half();
    dbg.ld_lq_f_hz = s_f_hz;
#if M1_LD_LQ_IDENT_INJECT_LUT_ENABLE
    ld_lq_sync_inject_deadband_profile();
#endif
}

static void ld_lq_enter_settle(float theta_el)
{
    ld_lq_grid_bias_from_idx(s_grid_idx, &s_id_bias, &s_iq_bias);
    if (s_grid_idx == 0u) {
        s_bias_ramp_en = 1u;
        s_id_ref_start = s_id_ref_cmd;
        s_iq_ref_start = s_iq_ref_cmd;
    } else {
        s_bias_ramp_en = 0u;
    s_coarse_l_valid = 0u;
        s_id_ref_cmd = s_id_bias;
        s_iq_ref_cmd = s_iq_bias;
        s_id_ref_start = s_id_bias;
        s_iq_ref_start = s_iq_bias;
    }
    s_sub = LD_LQ_SUB_SETTLE;
    s_settle_tick = 0u;
    s_settle_elapsed = 0u;
    s_u_inj_d = 0.0f;
    s_u_inj_q = 0.0f;
    s_theta_ref_el = theta_el;
    dbg.ld_lq_theta_drift_mech_deg = 0.0f;
#if M1_LD_LQ_IDENT_INJECT_LUT_ENABLE
    ld_lq_sync_inject_deadband_profile();
#endif
}

static void ld_lq_apply_bias_ramp(float *id_cmd, float *iq_cmd)
{
    float u;
    float ramp_s = M1_LD_LQ_IDENT_BIAS_RAMP_S;

    if (id_cmd == 0 || iq_cmd == 0) {
        return;
    }
    if (s_bias_ramp_en == 0u || ramp_s <= 0.0f) {
        *id_cmd = s_id_bias;
        *iq_cmd = s_iq_bias;
        return;
    }
    u = (float)s_settle_elapsed * M1_CTRL_TS_S / ramp_s;
    if (u > 1.0f) {
        u = 1.0f;
    }
    *id_cmd = s_id_ref_start + u * (s_id_bias - s_id_ref_start);
    *iq_cmd = s_iq_ref_start + u * (s_iq_bias - s_iq_ref_start);
}

static void ld_lq_advance_grid(float theta_el)
{
    s_grid_idx++;
    if (s_grid_idx >= M1_LD_LQ_GRID_N) {
        s_sub = LD_LQ_SUB_DONE;
        s_done = 1u;
        s_ok = 1u;
        ld_lq_update_result_ok();
        ld_lq_ident_sync_dbg();
#if M1_LD_LQ_IDENT_INJECT_LUT_ENABLE
        ld_lq_sync_inject_deadband_profile();
#endif
        return;
    }
    ld_lq_enter_settle(theta_el);
}

static uint8_t ld_lq_settle_tick(float id_fb, float iq_fb, float theta_el)
{
    const float bias_sum = fabsf(s_id_bias) + fabsf(s_iq_bias);
    const uint32_t need = ld_lq_ticks_from_s(M1_LD_LQ_IDENT_SETTLE_S);
    const uint32_t max_wait = ld_lq_ticks_from_s(M1_LD_LQ_IDENT_SETTLE_MAX_S);

    s_settle_elapsed++;

    if (ld_lq_check_theta_drift(theta_el)) {
        return 0u;
    }

    if (bias_sum <= M1_LD_LQ_IDENT_ZERO_BIAS_SUM_A) {
        return (s_settle_elapsed >= need) ? 1u : 0u;
    }

    if (fabsf(id_fb - s_id_bias) <= M1_LD_LQ_IDENT_EPS_TRACK_A &&
        fabsf(iq_fb - s_iq_bias) <= M1_LD_LQ_IDENT_EPS_TRACK_A) {
        s_settle_tick++;
    } else if (s_settle_tick > 0u) {
        s_settle_tick--;
    }

    if (s_settle_tick >= need) {
        return 1u;
    }
    return (s_settle_elapsed >= max_wait) ? 1u : 0u;
}

static uint8_t ld_lq_inject_active(void)
{
    return (s_sub == LD_LQ_SUB_INJ_LD || s_sub == LD_LQ_SUB_INJ_LQ) ? 1u : 0u;
}

#if M1_LD_LQ_IDENT_INJECT_LUT_ENABLE
static void ld_lq_sync_inject_deadband_profile(void)
{
    if (ld_lq_inject_active()) {
        deadband_service_apply_profile(DEADBAND_PROFILE_LUT_RUNTIME);
    } else {
        deadband_service_apply_profile(DEADBAND_PROFILE_OFF);
    }
}
#endif

#if M1_LD_LQ_IDENT_OPEN_LOOP_ENABLE
static void ld_lq_capture_u_bias(float ud, float uq)
{
    s_u_bias_d = ud;
    s_u_bias_q = uq;
}

static float ld_lq_u_bias_d(void)
{
    return s_u_bias_d;
}

static float ld_lq_u_bias_q(void)
{
    return s_u_bias_q;
}
#endif

static void ld_lq_on_axis_stage_done(uint8_t axis, float theta_el, float ud_pi, float uq_pi)
{
    float ud_b;
    float uq_b;

#if M1_LD_LQ_IDENT_OPEN_LOOP_ENABLE
    (void)ud_pi;
    (void)uq_pi;
    ud_b = s_u_bias_d;
    uq_b = s_u_bias_q;
#else
    ud_b = ud_pi;
    uq_b = uq_pi;
#endif

#if M1_LD_LQ_IDENT_FINE_ONLY
    if (s_inj_tier == LD_LQ_INJ_TIER_FINE) {
        ld_lq_store_axis_median(axis, LD_LQ_INJ_TIER_FINE);
        if (axis == 0u) {
#if M1_LD_LQ_IDENT_OPEN_LOOP_ENABLE
            ld_lq_capture_u_bias(ud_b, uq_b);
#endif
            ld_lq_begin_inject(1u, LD_LQ_INJ_TIER_FINE, ud_b, uq_b);
            s_sub = LD_LQ_SUB_INJ_LQ;
        } else {
            ld_lq_advance_grid(theta_el);
        }
        return;
    }
#else
    if (s_inj_tier == LD_LQ_INJ_TIER_COARSE) {
        ld_lq_capture_coarse_l();
        ld_lq_store_axis_coarse(axis);
        ld_lq_begin_inject(axis, LD_LQ_INJ_TIER_FINE, ud_b, uq_b);
        return;
    }
#endif
#if M1_LD_LQ_IDENT_F2_ENABLE
    if (s_inj_tier == LD_LQ_INJ_TIER_FINE) {
        ld_lq_store_axis_median(axis, LD_LQ_INJ_TIER_FINE);
        ld_lq_begin_inject(axis, LD_LQ_INJ_TIER_F2, ud_b, uq_b);
        return;
    }
    if (s_inj_tier == LD_LQ_INJ_TIER_F2) {
        ld_lq_store_axis_median(axis, LD_LQ_INJ_TIER_F2);
    } else
#endif
    {
        ld_lq_store_axis_median(axis, LD_LQ_INJ_TIER_FINE);
    }
    if (axis == 0u) {
#if M1_LD_LQ_IDENT_OPEN_LOOP_ENABLE
        ld_lq_capture_u_bias(ud_b, uq_b);
#endif
        ld_lq_begin_inject(1u, LD_LQ_INJ_TIER_COARSE, ud_b, uq_b);
        s_sub = LD_LQ_SUB_INJ_LQ;
    } else {
        ld_lq_advance_grid(theta_el);
    }
}

void ld_lq_ident_init(void)
{
    s_sub = LD_LQ_SUB_DONE;
    s_done = 1u;
    s_ok = 0u;
    s_grid_idx = 0u;
    s_theta_drift_max_el = M1_LD_LQ_IDENT_THETA_DRIFT_MECH_DEG * LD_LQ_PI / 180.0f *
                           (float)M1_POLE_PAIRS;
    s_f_hz = M1_LD_LQ_IDENT_FINE_ONLY ? M1_LD_LQ_IDENT_F_FINE_HZ : M1_LD_LQ_IDENT_F_COARSE_HZ;
    ld_lq_update_half_period();
    memset(&s_result, 0, sizeof(s_result));
    s_id_ref_cmd = 0.0f;
    s_iq_ref_cmd = 0.0f;
    s_id_ref_start = 0.0f;
    s_iq_ref_start = 0.0f;
    s_bias_ramp_en = 0u;
    s_coarse_l_valid = 0u;
#if M1_LD_LQ_MULTI_ANGLE_ENABLE
    s_angle_leg = 0u;
    memset(s_result_leg, 0, sizeof(s_result_leg));
    memset(s_theta_target_rad, 0, sizeof(s_theta_target_rad));
#endif
}

void ld_lq_ident_arm(float rs_ohm)
{
    s_sub = LD_LQ_SUB_SETTLE;
    s_done = 0u;
    s_ok = 0u;
    s_grid_idx = 0u;
    s_rs_ohm = (rs_ohm > 0.0f) ? rs_ohm : M1_RS_OHM;
    s_theta_drift_max_el = M1_LD_LQ_IDENT_THETA_DRIFT_MECH_DEG * LD_LQ_PI / 180.0f *
                           (float)M1_POLE_PAIRS;
    s_f_hz = M1_LD_LQ_IDENT_FINE_ONLY ? M1_LD_LQ_IDENT_F_FINE_HZ : M1_LD_LQ_IDENT_F_COARSE_HZ;
    ld_lq_update_half_period();
    memset(&s_result, 0, sizeof(s_result));
    s_result.rs_used_ohm = s_rs_ohm;
    s_coarse_l_valid = 0u;
    ld_lq_enter_settle(dbg.enc_theta_el);
}

void ld_lq_ident_first_grid_bias(float *id0, float *iq0)
{
    ld_lq_grid_bias_from_idx(0u, id0, iq0);
}

void ld_lq_ident_tick(float id_fb, float iq_fb, float ud_pi, float uq_pi,
                      float *id_ref_out, float *iq_ref_out)
{
    float id_cmd = s_id_bias;
    float iq_cmd = s_iq_bias;

    if (s_done) {
        s_u_inj_d = 0.0f;
        s_u_inj_q = 0.0f;
        s_id_ref_cmd = 0.0f;
        s_iq_ref_cmd = 0.0f;
        if (id_ref_out != 0) {
            *id_ref_out = 0.0f;
        }
        if (iq_ref_out != 0) {
            *iq_ref_out = 0.0f;
        }
        return;
    }

    switch (s_sub) {
    case LD_LQ_SUB_SETTLE:
        s_u_inj_d = 0.0f;
        s_u_inj_q = 0.0f;
        ld_lq_apply_bias_ramp(&id_cmd, &iq_cmd);
        if (ld_lq_settle_tick(id_fb, iq_fb, dbg.enc_theta_el)) {
#if M1_LD_LQ_IDENT_OPEN_LOOP_ENABLE
            ld_lq_capture_u_bias(ud_pi, uq_pi);
            ld_lq_begin_inject(0u, ld_lq_inj_tier_first(), s_u_bias_d, s_u_bias_q);
#else
            ld_lq_begin_inject(0u, ld_lq_inj_tier_first(), ud_pi, uq_pi);
#endif
            s_sub = LD_LQ_SUB_INJ_LD;
            id_cmd = s_id_bias;
            iq_cmd = s_iq_bias;
        }
        break;

    case LD_LQ_SUB_INJ_LD:
    case LD_LQ_SUB_INJ_LQ:
#if !M1_LD_LQ_IDENT_OPEN_LOOP_ENABLE
        s_ud_pi_cache = ud_pi;
        s_uq_pi_cache = uq_pi;
#endif
        ld_lq_inj_set_outputs();
        break;

    case LD_LQ_SUB_DONE:
    default:
        s_done = 1u;
        s_u_inj_d = 0.0f;
        s_u_inj_q = 0.0f;
        break;
    }

    s_id_ref_cmd = id_cmd;
    s_iq_ref_cmd = iq_cmd;
    if (id_ref_out != 0) {
        *id_ref_out = id_cmd;
    }
    if (iq_ref_out != 0) {
        *iq_ref_out = iq_cmd;
    }

    dbg.ld_lq_grid_idx = (float)s_grid_idx;
    dbg.ld_lq_id_bias = s_id_bias;
    dbg.ld_lq_iq_bias = s_iq_bias;
    dbg.ld_lq_v_inj = s_v_inj;
    dbg.ld_lq_axis = s_inj_axis;
    dbg.ld_lq_sub = (uint8_t)s_sub;
    dbg.ld_lq_f_hz = s_f_hz;
}

void ld_lq_ident_integrate(float id_fb, float iq_fb, float ud_out, float uq_out)
{
    const uint8_t axis = (s_sub == LD_LQ_SUB_INJ_LQ) ? 1u : 0u;

    if (!ld_lq_inject_active() || s_done) {
        return;
    }

    if (ld_lq_check_theta_drift(dbg.enc_theta_el)) {
        return;
    }

    if (s_half_tick == 0u && s_inj_half == LD_INJ_HALF_POS && s_inj_have_pos == 0u) {
        s_i_start = ld_lq_axis_i(id_fb, iq_fb);
    }

    if (ld_lq_try_finish_half(id_fb, iq_fb, ud_out, uq_out)) {
#if M1_LD_LQ_IDENT_OPEN_LOOP_ENABLE
        ld_lq_on_axis_stage_done(axis, dbg.enc_theta_el, s_u_bias_d, s_u_bias_q);
#else
        ld_lq_on_axis_stage_done(axis, dbg.enc_theta_el, s_ud_pi_cache, s_uq_pi_cache);
#endif
    }
}

float ld_lq_ident_u_inj_d(void)
{
    return s_u_inj_d;
}

float ld_lq_ident_u_inj_q(void)
{
    return s_u_inj_q;
}

uint8_t ld_lq_ident_inject_active(void)
{
    return ld_lq_inject_active();
}

uint8_t ld_lq_ident_pi_active(void)
{
    if (s_done) {
        return 0u;
    }
#if M1_LD_LQ_IDENT_OPEN_LOOP_ENABLE
    return ld_lq_inject_active() ? 0u : 1u;
#else
    return 1u;
#endif
}

#if M1_LD_LQ_IDENT_OPEN_LOOP_ENABLE
float ld_lq_ident_u_bias_d(void)
{
    return ld_lq_u_bias_d();
}

float ld_lq_ident_u_bias_q(void)
{
    return ld_lq_u_bias_q();
}
#endif

uint8_t ld_lq_ident_is_done(void)
{
    return s_done;
}

uint8_t ld_lq_ident_is_ok(void)
{
    return s_ok;
}

uint8_t ld_lq_ident_open_seq_phase(void)
{
    if (!s_done) {
        return 57u;
    }
    return s_ok ? 58u : 59u;
}

void ld_lq_ident_get_result(ld_lq_ident_result_t *out)
{
    if (out != 0) {
        *out = s_result;
    }
}

static void ld_lq_sync_proc_dbg(void)
{
    dbg.ld_lq_proc_grid = (float)s_grid_idx;
    dbg.ld_lq_proc_amp_idx = (float)s_amp_idx;
    dbg.ld_lq_proc_coarse_l_uH = s_coarse_l_h * 1e6f;
    dbg.ld_lq_proc_half_ticks = (float)s_half_period_ticks;
    dbg.ld_lq_proc_code = (float)((uint32_t)s_sub * 100u + (uint32_t)s_inj_axis * 10u +
                                  ld_lq_proc_tier_band(s_inj_tier) * 5u +
                                  (uint32_t)s_amp_idx);
}

static void ld_lq_sync_dbg_common(void)
{
    uint8_t gi;

    dbg.ld_lq_grid_idx = (float)s_grid_idx;
    dbg.ld_lq_id_bias = s_id_bias;
    dbg.ld_lq_iq_bias = s_iq_bias;
    dbg.ld_lq_v_inj = s_v_inj;
    dbg.ld_lq_axis = s_inj_axis;
    dbg.ld_lq_sub = (uint8_t)s_sub;
    dbg.ld_lq_rs_used = s_result.rs_used_ohm;
    dbg.ld_lq_n_ld_ok = (float)s_result.n_ld_ok;
    dbg.ld_lq_n_lq_ok = (float)s_result.n_lq_ok;
    dbg.ld_lq_ok = s_result.ok;
    dbg.ld_lq_f_hz = s_f_hz;
    ld_lq_sync_proc_dbg();

    gi = (s_grid_idx > 0u) ? (uint8_t)(s_grid_idx - 1u) : 0u;
    if (s_amp_l_valid_n > 0u) {
        dbg.ld_lq_L_est_uH = s_amp_l_buf[s_amp_l_valid_n - 1u] * 1e6f;
    } else if (s_inj_tier == LD_LQ_INJ_TIER_FINE && s_result.lq_valid_fine[gi]) {
        dbg.ld_lq_L_est_uH = s_result.lq_h_fine[gi] * 1e6f;
    } else if (s_inj_tier == LD_LQ_INJ_TIER_FINE && s_result.ld_valid_fine[gi]) {
        dbg.ld_lq_L_est_uH = s_result.ld_h_fine[gi] * 1e6f;
#if M1_LD_LQ_IDENT_F2_ENABLE
    } else if (s_inj_tier == LD_LQ_INJ_TIER_F2 && s_result.lq_valid_f2[gi]) {
        dbg.ld_lq_L_est_uH = s_result.lq_h_f2[gi] * 1e6f;
    } else if (s_inj_tier == LD_LQ_INJ_TIER_F2 && s_result.ld_valid_f2[gi]) {
        dbg.ld_lq_L_est_uH = s_result.ld_h_f2[gi] * 1e6f;
#endif
    } else if (s_coarse_l_valid != 0u) {
        dbg.ld_lq_L_est_uH = s_coarse_l_h * 1e6f;
    } else if (s_result.lq_valid[gi]) {
        dbg.ld_lq_L_est_uH = s_result.lq_h[gi] * 1e6f;
    } else if (s_result.ld_valid[gi]) {
        dbg.ld_lq_L_est_uH = s_result.ld_h[gi] * 1e6f;
    } else {
        dbg.ld_lq_L_est_uH = 0.0f;
    }
}

void ld_lq_ident_sync_dbg(void)
{
    ld_lq_sync_dbg_common();
#if M1_LD_LQ_MULTI_ANGLE_ENABLE
    dbg.ld_lq_angle_leg = s_angle_leg;
    if (s_angle_leg < M1_LD_LQ_IDENT_ANGLE_COUNT) {
        dbg.ld_lq_theta_target_el = s_theta_target_rad[s_angle_leg];
    }
#endif
}

#if M1_LD_LQ_MULTI_ANGLE_ENABLE
void ld_lq_ident_set_angle_leg(uint8_t leg, float theta_el_rad)
{
    if (leg >= M1_LD_LQ_IDENT_ANGLE_COUNT) {
        leg = (uint8_t)(M1_LD_LQ_IDENT_ANGLE_COUNT - 1u);
    }
    s_angle_leg = leg;
    s_theta_target_rad[leg] = theta_el_rad;
    dbg.ld_lq_angle_leg = leg;
    dbg.ld_lq_theta_target_el = theta_el_rad;
}

void ld_lq_ident_commit_leg(uint8_t leg)
{
    if (leg < M1_LD_LQ_IDENT_ANGLE_COUNT) {
        s_result_leg[leg] = s_result;
        s_result_leg[leg].ok = s_result.ok;
    }
}

void ld_lq_ident_get_result_leg(uint8_t leg, ld_lq_ident_result_t *out)
{
    if (out == 0) {
        return;
    }
    if (leg < M1_LD_LQ_IDENT_ANGLE_COUNT) {
        *out = s_result_leg[leg];
    } else {
        *out = (ld_lq_ident_result_t){0};
    }
}

uint8_t ld_lq_ident_angle_leg(void)
{
    return s_angle_leg;
}
#endif /* M1_LD_LQ_MULTI_ANGLE_ENABLE */

#endif /* M1_LD_LQ_IDENT_ENABLE */
