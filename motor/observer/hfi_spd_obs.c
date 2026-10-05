/**
 * @file hfi_spd_obs.c
 * @brief P5: HFI helpers moved from motor_current.c (call order unchanged).
 */
#include "observer/obs_cfg.h"
#include "motor_math.h"
#include "motor_context.h"
#include "motor_trig.h"
#include "dbg_monitor.h"
#include "foc_pi.h"
#include "observer/hfi_sqwave.h"
#include "observer/emf_pll.h"
#include "observer/hfi_current_priv.h"

#if (M1_HFI_GATE == 78) || (M1_HFI_GATE == 79) || (M1_HFI_GATE == 80)
#ifndef M1_HFI_VESC_WIN_HI_RPM
#define M1_HFI_VESC_WIN_HI_RPM          (1050.0f)
#endif
#ifndef M1_HFI_VESC_WIN_LO_RPM
#define M1_HFI_VESC_WIN_LO_RPM          (950.0f)
#endif
/** ≥HI 置 want_smo，≤LO 清零（VESC 式滞回）。S1 观察；S2 硬关；S2b 软交接。 */
static uint8_t s_vesc_win_smo;

uint8_t hfi_vesc_win_smo(void)
{
    return s_vesc_win_smo;
}

void hfi_vesc_win_smo_set(uint8_t v)
{
    s_vesc_win_smo = v;
}
#if M1_HFI_GATE == 80
static uint8_t s_vesc_ho_active;
static uint32_t s_vesc_ho_n;

uint8_t hfi_vesc_ho_active(void)
{
    return s_vesc_ho_active;
}

uint32_t hfi_vesc_ho_n(void)
{
    return s_vesc_ho_n;
}

void hfi_vesc_ho_set(uint8_t active, uint32_t n)
{
    s_vesc_ho_active = active;
    s_vesc_ho_n = n;
}
#endif

void hfi_vesc_win_obs_update(float w_rpm)
{
    float aw = w_rpm;

    if (aw < 0.0f) {
        aw = -aw;
    }
    if (s_vesc_win_smo != 0u) {
        if (aw <= M1_HFI_VESC_WIN_LO_RPM) {
            s_vesc_win_smo = 0u;
        }
    } else if (aw >= M1_HFI_VESC_WIN_HI_RPM) {
        s_vesc_win_smo = 1u;
    }
    dbg.hfi_vesc_win_smo = (float)s_vesc_win_smo;
}
#endif
#if M1_HFI_ENABLE && M1_HFI_MOTION_BYPASS_ENABLE && M1_EMF_SMO_ENABLE && \
    M1_EMF_PLL_ENABLE
/* 20 ms 滑动平均。0 kHz 。400 拍。交接看这个，不看含 Kp·ε 的瞬。ω。*/
#ifndef HFI_SMO_W_MA_N
#define HFI_SMO_W_MA_N 400u
#endif
static float s_smo_w_hist[HFI_SMO_W_MA_N];
static float s_smo_w_sum;
static uint16_t s_smo_w_i;
static uint16_t s_smo_w_fill;

void hfi_smo_w_ma_reset(void)
{
    s_smo_w_sum = 0.0f;
    s_smo_w_i = 0u;
    s_smo_w_fill = 0u;
}

float hfi_smo_w_ma_step(float rpm)
{
    if (s_smo_w_fill >= HFI_SMO_W_MA_N) {
        s_smo_w_sum -= s_smo_w_hist[s_smo_w_i];
    } else {
        s_smo_w_fill++;
    }
    s_smo_w_hist[s_smo_w_i] = rpm;
    s_smo_w_sum += rpm;
    s_smo_w_i++;
    if (s_smo_w_i >= HFI_SMO_W_MA_N) {
        s_smo_w_i = 0u;
    }
    return s_smo_w_sum / (float)s_smo_w_fill;
}
#endif
#if ((M1_HFI_GATE == 38) || (M1_HFI_GATE == 53) || (M1_HFI_GATE == 54) || \
     (M1_HFI_GATE == 55) || (M1_HFI_GATE == 56) || (M1_HFI_GATE == 57) || (M1_HFI_GATE == 58) || (M1_HFI_GATE == 59) || (M1_HFI_GATE == 60) || (M1_HFI_GATE == 61) || (M1_HFI_GATE == 62) || (M1_HFI_GATE == 63) || (M1_HFI_GATE == 64) || (M1_HFI_GATE == 65) || (M1_HFI_GATE == 66) || (M1_HFI_GATE == 67) || (M1_HFI_GATE == 68) || (M1_HFI_GATE == 69) || (M1_HFI_GATE == 70) || (M1_HFI_GATE == 71) || (M1_HFI_GATE == 72) || (M1_HFI_GATE == 73) || (M1_HFI_GATE == 74) || (M1_HFI_GATE == 75) || (M1_HFI_GATE == 76) || (M1_HFI_GATE == 77) || (M1_HFI_GATE == 78) || (M1_HFI_GATE == 79) || (M1_HFI_GATE == 80) || (M1_HFI_GATE == 91) || (M1_HFI_GATE == 92) || (M1_HFI_GATE == 93)) && \
    M1_SPEED_LOOP_ENABLE
/**
 * @brief 电流环出力时，用速度环同一。PI 。Iq，不。iq_ref。
 * @note ω* 。whfi 。150 ms 低通。速度环不跟电流环带宽，毛刺不进指令。
 *       影子环没有「Iq 改变转速」的反馈，积分按 1 s 泄回维持馈流的平衡，
 *       否则大约 1 rpm 的平均差也会在几秒内打满。。 rpm 以内不进积分。
 */
void hfi_spd_shadow_step(float iq_hold, float w_fb_rpm)
{
    static foc_pi_t s_pi;
    static uint8_t s_on;
    static uint8_t s_div;
    static float s_ref;
    static float s_iq_arm;
    const float tau_ref_s = 0.15f;
    const float tau_leak_s = 1.0f;
    const float dead_rpm = 8.0f;
    float err;
    float err_i;
    float p_term;
    float i_ss;
    float leak;
    float out;

    if (hfi_sqwave_get_stage() != HFI_STAGE_RUN) {
        s_on = 0u;
        s_div = 0u;
        dbg.hfi_iq_spd_shadow = 0.0f;
        return;
    }
    if (++s_div < M1_SPEED_DECIM) {
        return;
    }
    s_div = 0u;
    if (s_on == 0u) {
        if ((iq_hold > 0.4f) || (iq_hold < -0.4f)) {
            foc_pi_init(&s_pi, M1_SPEED_PI_KP, M1_SPEED_PI_KI,
                        M1_SPEED_PI_OUT_MIN, M1_SPEED_PI_OUT_MAX,
                        M1_SPEED_PI_INT_MIN, M1_SPEED_PI_INT_MAX);
            s_ref = w_fb_rpm;
            s_iq_arm = iq_hold;
            foc_pi_bumpless_beta(&s_pi, iq_hold, s_ref, w_fb_rpm,
                                 M1_SPEED_PI_BETA);
            s_on = 1u;
            dbg.hfi_iq_spd_shadow = iq_hold;
        } else {
            dbg.hfi_iq_spd_shadow = 0.0f;
        }
        return;
    }

    s_ref += (M1_SPEED_TS_S / (tau_ref_s + M1_SPEED_TS_S)) * (w_fb_rpm - s_ref);
    err = s_ref - w_fb_rpm;
    err_i = err;
    if (err_i > dead_rpm) {
        err_i -= dead_rpm;
    } else if (err_i < -dead_rpm) {
        err_i += dead_rpm;
    } else {
        err_i = 0.0f;
    }
    s_pi.integrator += s_pi.ki * err_i;
    /* 平衡点：fb=ω* 时输出仍等于馈流，补。β 随转速变化的那一。*/
    i_ss = s_iq_arm - s_pi.kp * (M1_SPEED_PI_BETA - 1.0f) * s_ref;
    leak = M1_SPEED_TS_S / tau_leak_s;
    s_pi.integrator += (i_ss - s_pi.integrator) * leak;
    if (s_pi.integrator > s_pi.int_max) {
        s_pi.integrator = s_pi.int_max;
    } else if (s_pi.integrator < s_pi.int_min) {
        s_pi.integrator = s_pi.int_min;
    }
    p_term = s_pi.kp * (M1_SPEED_PI_BETA * s_ref - w_fb_rpm);
    out = p_term + s_pi.integrator;
    if (out > s_pi.out_max) {
        out = s_pi.out_max;
    } else if (out < s_pi.out_min) {
        out = s_pi.out_min;
    }
    dbg.hfi_iq_spd_shadow = out;
}
#endif
