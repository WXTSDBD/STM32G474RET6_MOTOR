/**
 * @file motor_current.c
 * @brief M1 JEOC：�?�?采样 �?Park �?foc_loop �?SVPWM �?kick �?telem�?
 */

#include "motor_current.h"

#include <stddef.h>

#include "encoder.h"
#include "app_uart_dma_debug.h"
#include "deadband_flow.h"
#include "deadband_id_cal.h"
#include "deadband_module.h"
#include "dbg_monitor.h"
#include "foc_pi.h"
#include "foc_svpwm.h"
#include "ld_lq_ident.h"
#include "motor_foc_loop.h"
#include "motor_open_sweep.h"
#include "motor_outer_loop.h"
#include "motor_phase_binding.h"
#include "motor_params_m1.h"
#if M1_PLL_ENABLE
#include "motor_pll.h"
#endif
#include "motor_startup.h"
#include "motor_if.h"
#include "motor_trig.h"
#if M1_EMF_VEQ_ENABLE
#include "observer/emf_veq.h"
#endif
#if M1_EMF_SMO_ENABLE
#include "observer/emf_smo.h"
#endif
#if M1_EMF_PLL_ENABLE
#include "observer/emf_pll.h"
#endif
#if M1_OBS_SOFT_SWITCH_ENABLE
#include "observer/obs_soft_switch.h"
#include "observer/obs_theta_notch.h"
#endif
#if M1_HFI_ENABLE
#include "observer/hfi_sqwave.h"
#ifndef M1_HFI_INJECT_AB_ENABLE
#define M1_HFI_INJECT_AB_ENABLE         0
#endif
#ifndef M1_HFI_INJECT_POST_LOOP
#define M1_HFI_INJECT_POST_LOOP         0
#endif
#ifndef M1_HFI_DEMOD_INJ_AXIS
#define M1_HFI_DEMOD_INJ_AXIS           0
#endif
#endif
#if M1_IDENT_ENABLE
#include "ident_flow.h"
#endif
#if M1_SPEED_IDENT_ENABLE
#include "speed_ident_flow.h"
#include "speed_ident_module.h"
#endif

static motor_context_t s_m1_ctx;

#if M1_EMF_VEQ_ENABLE
static emf_veq_t s_emf_veq;
#endif
#if M1_EMF_SMO_ENABLE
static emf_smo_t s_emf_smo;
#endif
#if M1_EMF_PLL_ENABLE
static emf_pll_t s_emf_pll;
#endif
#if M1_OBS_SPD_PLL_ENABLE && M1_PLL_ENABLE
/** 电角域：�?EMF-PLL θ̂ �?机械 rpm�? kHz 更新；OBS 时进速度�?*/
static motor_pll_t s_obs_spd_pll;
static uint8_t s_obs_spd_pll_primed;
static uint16_t s_obs_spd_div;
static float s_obs_spd_pll_rpm;
static float s_obs_spd_fb_lpf_rpm;
static uint8_t s_obs_spd_fb_lpf_on;
#endif
#if M1_PLL_ENABLE
/** 外环实际速度反馈（有�?编码�?PLL；OBS+速切=观测 PLL�?*/
static float s_speed_fb_rpm;
#endif

#if (M1_HFI_GATE == 132) || (M1_HFI_GATE == 133) || (M1_HFI_GATE == 134) || (M1_HFI_GATE == 135) || (M1_HFI_GATE == 136) || (M1_HFI_GATE == 137)
/* 速度环反馈上的两个陷波器。不进角度 PLL。指令 40..250 rpm 才开。 */
static uint8_t s_spd_notch_on;
static float s_notch_cmd = -1.0f;
static float s_n12_b0, s_n12_b1, s_n12_b2, s_n12_a1, s_n12_a2;
static float s_n12_x1, s_n12_x2, s_n12_y1, s_n12_y2;
static float s_n14_b0, s_n14_b1, s_n14_b2, s_n14_a1, s_n14_a2;
static float s_n14_x1, s_n14_x2, s_n14_y1, s_n14_y2;

static void hfi_notch_coeff(float f_hz, float q, float fs,
                            float *b0, float *b1, float *b2,
                            float *a1, float *a2)
{
    float w = 6.28318530718f * f_hz / fs;
    float w2 = w * w;
    float cw = 1.0f - (0.5f * w2);
    float sw = w * (1.0f - (w2 * (1.0f / 6.0f)));
    float alpha = sw / (2.0f * q);
    float inv = 1.0f / (1.0f + alpha);

    *b0 = inv;
    *b1 = (-2.0f * cw) * inv;
    *b2 = inv;
    *a1 = (-2.0f * cw) * inv;
    *a2 = (1.0f - alpha) * inv;
}

static void hfi_notch_prime(float x, float *x1, float *x2, float *y1, float *y2)
{
    *x1 = x;
    *x2 = x;
    *y1 = x;
    *y2 = x;
}

static float hfi_notch_run(float x,
                           float b0, float b1, float b2, float a1, float a2,
                           float *x1, float *x2, float *y1, float *y2)
{
    float y = (b0 * x) + (b1 * (*x1)) + (b2 * (*x2)) - (a1 * (*y1)) - (a2 * (*y2));

    *x2 = *x1;
    *x1 = x;
    *y2 = *y1;
    *y1 = y;
    return y;
}

static float hfi_spd_notch_rpm(float rpm, float cmd)
{
    const float fs = 1.0f / M1_CTRL_TS_S;
    float cmd_abs = cmd;
    float df;

    if (cmd_abs < 0.0f) {
        cmd_abs = -cmd_abs;
    }
    if ((cmd_abs < 40.0f) || (cmd_abs > 250.0f)) {
        s_spd_notch_on = 0u;
        s_notch_cmd = cmd_abs;
        hfi_notch_prime(rpm, &s_n12_x1, &s_n12_x2, &s_n12_y1, &s_n12_y2);
        hfi_notch_prime(rpm, &s_n14_x1, &s_n14_x2, &s_n14_y1, &s_n14_y2);
        return rpm;
    }
    df = cmd_abs - s_notch_cmd;
    if (df < 0.0f) {
        df = -df;
    }
    if ((s_spd_notch_on == 0u) || (df > 0.5f)) {
        hfi_notch_coeff(12.0f * cmd_abs / 60.0f, 8.0f, fs,
                        &s_n12_b0, &s_n12_b1, &s_n12_b2, &s_n12_a1, &s_n12_a2);
        hfi_notch_coeff(14.0f * cmd_abs / 60.0f, 8.0f, fs,
                        &s_n14_b0, &s_n14_b1, &s_n14_b2, &s_n14_a1, &s_n14_a2);
        if (s_spd_notch_on == 0u) {
            hfi_notch_prime(rpm, &s_n12_x1, &s_n12_x2, &s_n12_y1, &s_n12_y2);
            hfi_notch_prime(rpm, &s_n14_x1, &s_n14_x2, &s_n14_y1, &s_n14_y2);
        }
        s_notch_cmd = cmd_abs;
        s_spd_notch_on = 1u;
    }
    rpm = hfi_notch_run(rpm, s_n12_b0, s_n12_b1, s_n12_b2, s_n12_a1, s_n12_a2,
                        &s_n12_x1, &s_n12_x2, &s_n12_y1, &s_n12_y2);
    return hfi_notch_run(rpm, s_n14_b0, s_n14_b1, s_n14_b2, s_n14_a1, s_n14_a2,
                         &s_n14_x1, &s_n14_x2, &s_n14_y1, &s_n14_y2);
}
#endif

#if M1_HFI_GATE == 140
/* 100 rpm 指令才开。按实测转速陷掉每转 24 次和 28 次，12 次和 14 次留着。 */
static uint8_t s_n24_on;
static float s_n24_rpm = -1.0f;
static float s_n24_abs;
static float s_n24_b0, s_n24_b1, s_n24_b2, s_n24_a1, s_n24_a2;
static float s_n24_x1, s_n24_x2, s_n24_y1, s_n24_y2;
static float s_n28_b0, s_n28_b1, s_n28_b2, s_n28_a1, s_n28_a2;
static float s_n28_x1, s_n28_x2, s_n28_y1, s_n28_y2;

static void hfi_n24_coeff(float f_hz, float q, float fs,
                          float *b0, float *b1, float *b2,
                          float *a1, float *a2)
{
    float w = 6.28318530718f * f_hz / fs;
    float w2 = w * w;
    float cw = 1.0f - (0.5f * w2);
    float sw = w * (1.0f - (w2 * (1.0f / 6.0f)));
    float alpha = sw / (2.0f * q);
    float inv = 1.0f / (1.0f + alpha);

    *b0 = inv;
    *b1 = (-2.0f * cw) * inv;
    *b2 = inv;
    *a1 = (-2.0f * cw) * inv;
    *a2 = (1.0f - alpha) * inv;
}

static void hfi_n24_prime(float x, float *x1, float *x2, float *y1, float *y2)
{
    *x1 = x;
    *x2 = x;
    *y1 = x;
    *y2 = x;
}

static float hfi_n24_run(float x,
                         float b0, float b1, float b2, float a1, float a2,
                         float *x1, float *x2, float *y1, float *y2)
{
    float y = (b0 * x) + (b1 * (*x1)) + (b2 * (*x2)) - (a1 * (*y1)) - (a2 * (*y2));

    *x2 = *x1;
    *x1 = x;
    *y2 = *y1;
    *y1 = y;
    return y;
}

static float hfi_spd_notch24_rpm(float rpm, float cmd)
{
    const float fs = 1.0f / M1_CTRL_TS_S;
    float cmd_abs = cmd;
    float w;
    float df;
    const float a = M1_CTRL_TS_S / (0.15f + M1_CTRL_TS_S);

    if (cmd_abs < 0.0f) {
        cmd_abs = -cmd_abs;
    }
    w = rpm;
    if (w < 0.0f) {
        w = -w;
    }
    s_n24_abs += a * (w - s_n24_abs);
    if ((cmd_abs < 80.0f) || (cmd_abs > 130.0f) || (s_n24_abs < 40.0f)) {
        s_n24_on = 0u;
        s_n24_rpm = -1.0f;
        hfi_n24_prime(rpm, &s_n24_x1, &s_n24_x2, &s_n24_y1, &s_n24_y2);
        hfi_n24_prime(rpm, &s_n28_x1, &s_n28_x2, &s_n28_y1, &s_n28_y2);
        return rpm;
    }
    df = s_n24_abs - s_n24_rpm;
    if (df < 0.0f) {
        df = -df;
    }
    if ((s_n24_on == 0u) || (df > 0.5f)) {
        hfi_n24_coeff(24.0f * s_n24_abs / 60.0f, 8.0f, fs,
                      &s_n24_b0, &s_n24_b1, &s_n24_b2, &s_n24_a1, &s_n24_a2);
        hfi_n24_coeff(28.0f * s_n24_abs / 60.0f, 8.0f, fs,
                      &s_n28_b0, &s_n28_b1, &s_n28_b2, &s_n28_a1, &s_n28_a2);
        if (s_n24_on == 0u) {
            hfi_n24_prime(rpm, &s_n24_x1, &s_n24_x2, &s_n24_y1, &s_n24_y2);
            hfi_n24_prime(rpm, &s_n28_x1, &s_n28_x2, &s_n28_y1, &s_n28_y2);
        }
        s_n24_rpm = s_n24_abs;
        s_n24_on = 1u;
    }
    rpm = hfi_n24_run(rpm, s_n24_b0, s_n24_b1, s_n24_b2, s_n24_a1, s_n24_a2,
                      &s_n24_x1, &s_n24_x2, &s_n24_y1, &s_n24_y2);
    return hfi_n24_run(rpm, s_n28_b0, s_n28_b1, s_n28_b2, s_n28_a1, s_n28_a2,
                       &s_n28_x1, &s_n28_x2, &s_n28_y1, &s_n28_y2);
}
#endif

#if M1_HFI_GATE == 134
/* 实测转速 0..200 rpm：给速度反馈补 5 ms 低通在齿槽频率上的相位。不跟指令。 */
static float s_lead_rpm;
static float s_lead_rep;
static float s_ld_b0, s_ld_b1, s_ld_a1;
static float s_ld_x1, s_ld_y1;
static float s_ld_f = -1.0f;
static uint8_t s_ld_on;

static float hfi_lead_sqrt(float x)
{
    float y;
    int i;

    if (x <= 0.0f) {
        return 0.0f;
    }
    y = (x > 1.0f) ? x : 1.0f;
    for (i = 0; i < 4; i++) {
        y = 0.5f * (y + (x / y));
    }
    return y;
}

static void hfi_lead_prime(float x)
{
    s_ld_x1 = x;
    s_ld_y1 = x;
}

static void hfi_lead_coeff(float f_hz)
{
    const float fs = 1.0f / M1_CTRL_TS_S;
    const float w0 = 6.28318530718f * f_hz;
    float x = w0 * 0.005f;
    float phi;
    float sp;
    float alpha;
    float root;
    float wz;
    float wp;
    float c;
    float a0;

    if (x > 1.0f) {
        const float xi = 1.0f / x;

        phi = 1.5707963f - ((0.78539816f * xi) + (0.273f * xi * (1.0f - xi)));
    } else {
        phi = (0.78539816f * x) + (0.273f * x * (1.0f - x));
    }
    if (phi > 0.5235988f) {
        phi = 0.5235988f;
    }
    sp = phi - ((phi * phi * phi) * (1.0f / 6.0f));
    alpha = (1.0f + sp) / (1.0f - sp);
    root = hfi_lead_sqrt(alpha);
    wz = w0 / root;
    wp = w0 * root;
    c = 2.0f * fs;
    a0 = 1.0f + (c / wp);
    s_ld_b0 = (1.0f + (c / wz)) / a0;
    s_ld_b1 = (1.0f - (c / wz)) / a0;
    s_ld_a1 = (1.0f - (c / wp)) / a0;
    s_ld_f = f_hz;
}

static float hfi_spd_lead_rpm(float rpm, float cmd)
{
    float wabs = rpm;
    float g;
    float led;
    float notched;
    float f0;
    float df;

    if (wabs < 0.0f) {
        wabs = -wabs;
    }
    s_lead_rpm += (M1_CTRL_TS_S / 0.15f) * (wabs - s_lead_rpm);
    if (s_lead_rpm >= 240.0f) {
        g = 0.0f;
    } else if (s_lead_rpm <= 200.0f) {
        g = 1.0f;
    } else {
        g = (240.0f - s_lead_rpm) / 40.0f;
    }
    s_lead_rep = s_lead_rpm * g;
    f0 = 13.0f * s_lead_rpm / 60.0f;
    if (f0 < 1.5f) {
        s_ld_on = 0u;
        s_ld_f = -1.0f;
        hfi_lead_prime(rpm);
        led = rpm;
    } else {
        df = f0 - s_ld_f;
        if (df < 0.0f) {
            df = -df;
        }
        if ((s_ld_on == 0u) || (df > 0.4f)) {
            hfi_lead_coeff(f0);
            if (s_ld_on == 0u) {
                hfi_lead_prime(rpm);
            }
            s_ld_on = 1u;
        }
        led = (s_ld_b0 * rpm) + (s_ld_b1 * s_ld_x1) - (s_ld_a1 * s_ld_y1);
        s_ld_x1 = rpm;
        s_ld_y1 = led;
    }
    notched = hfi_spd_notch_rpm(rpm, cmd);
    return ((1.0f - g) * notched) + (g * led);
}
#endif

#if (M1_HFI_GATE == 135) || (M1_HFI_GATE == 136) || (M1_HFI_GATE == 137)
/* 实测 60..140 rpm：按角度超前，增益为 1。以外走陷波器。
 * 136：乘 sin/cos 之前先减去 150 ms 平均转速。只转每转 12 次和 14 次。
 * 137：同 136，再转每转 24 次。 */
static float s_ph_rpm;
static float s_ph_rep;
#if (M1_HFI_GATE == 136) || (M1_HFI_GATE == 137)
static float s_ph_dc;
#endif
static float s_ph12_c;
static float s_ph12_s;
static float s_ph14_c;
static float s_ph14_s;
#if M1_HFI_GATE == 137
static float s_ph24_c;
static float s_ph24_s;
#endif

static float hfi_ph_wrap(float x)
{
    const float pi = 3.14159265f;
    const float twopi = 6.2831853f;

    while (x > pi) {
        x -= twopi;
    }
    while (x < -pi) {
        x += twopi;
    }
    return x;
}

static float hfi_ph_atan(float x)
{
    float phi;

    if (x > 1.0f) {
        const float xi = 1.0f / x;

        phi = 1.5707963f - ((0.78539816f * xi) + (0.273f * xi * (1.0f - xi)));
    } else {
        phi = (0.78539816f * x) + (0.273f * x * (1.0f - x));
    }
    if (phi > 0.5235988f) {
        phi = 0.5235988f;
    }
    return phi;
}

static void hfi_ph_pair(float rpm, float p, float phi,
                        float *c, float *s, float *h, float *hadv)
{
    const float a = M1_CTRL_TS_S / 0.25f;
    float co;
    float si;
    float cf;
    float sf;
    float cr;
    float sr;

    motor_trig_sincos(p, &co, &si);
    *c += a * ((rpm * co) - *c);
    *s += a * ((rpm * si) - *s);
    cr = 2.0f * (*c);
    sr = 2.0f * (*s);
    *h = (cr * co) + (sr * si);
    motor_trig_sincos(phi, &cf, &sf);
    *hadv = (((cr * cf) + (sr * sf)) * co) + ((((-cr) * sf) + (sr * cf)) * si);
}

static float hfi_spd_phase_rpm(float rpm, float cmd)
{
    const float th = hfi_sqwave_get_theta_hat();
    const float pp = (float)M1_POLE_PAIRS;
    float wabs = rpm;
    float g;
    float h12;
    float h12a;
    float h14;
    float h14a;
#if M1_HFI_GATE == 137
    float h24;
    float h24a;
#endif
    float led;
    float notched;

    if (wabs < 0.0f) {
        wabs = -wabs;
    }
    s_ph_rpm += (M1_CTRL_TS_S / 0.15f) * (wabs - s_ph_rpm);
    if ((s_ph_rpm <= 40.0f) || (s_ph_rpm >= 160.0f)) {
        g = 0.0f;
    } else if (s_ph_rpm < 60.0f) {
        g = (s_ph_rpm - 40.0f) / 20.0f;
    } else if (s_ph_rpm > 140.0f) {
        g = (160.0f - s_ph_rpm) / 20.0f;
    } else {
        g = 1.0f;
    }
    s_ph_rep = s_ph_rpm * g;
    {
        float xdem = rpm;

#if (M1_HFI_GATE == 136) || (M1_HFI_GATE == 137)
        s_ph_dc += (M1_CTRL_TS_S / 0.15f) * (rpm - s_ph_dc);
        xdem = rpm - s_ph_dc;
#endif
        hfi_ph_pair(xdem, hfi_ph_wrap(th * (12.0f / pp)),
                    hfi_ph_atan(6.28318530718f * (12.0f * s_ph_rpm / 60.0f) * 0.005f),
                    &s_ph12_c, &s_ph12_s, &h12, &h12a);
        hfi_ph_pair(xdem, hfi_ph_wrap(th * (14.0f / pp)),
                    hfi_ph_atan(6.28318530718f * (14.0f * s_ph_rpm / 60.0f) * 0.005f),
                    &s_ph14_c, &s_ph14_s, &h14, &h14a);
#if M1_HFI_GATE == 137
        hfi_ph_pair(xdem, hfi_ph_wrap(th * (24.0f / pp)),
                    hfi_ph_atan(6.28318530718f * (24.0f * s_ph_rpm / 60.0f) * 0.005f),
                    &s_ph24_c, &s_ph24_s, &h24, &h24a);
#endif
    }
#if M1_HFI_GATE == 137
    led = (rpm - h12 - h14 - h24) + h12a + h14a + h24a;
#else
    led = (rpm - h12 - h14) + h12a + h14a;
#endif
    notched = hfi_spd_notch_rpm(rpm, cmd);
    return ((1.0f - g) * notched) + (g * led);
}
#endif

#if M1_HFI_GATE == 133
/* 100 rpm 档的齿槽补偿。更高转速和 0 rpm 不加。 */
static float s_rip12_c;
static float s_rip12_s;
static float s_rip14_c;
static float s_rip14_s;
static float s_rip_amp;

static float hfi_rip_wrap(float x)
{
    const float pi = 3.14159265f;
    const float twopi = 6.2831853f;

    while (x > pi) {
        x -= twopi;
    }
    while (x < -pi) {
        x += twopi;
    }
    return x;
}

static float hfi_rip_sqrt(float x)
{
    float y;
    int i;

    if (x <= 0.0f) {
        return 0.0f;
    }
    y = (x > 1.0f) ? x : 1.0f;
    for (i = 0; i < 4; i++) {
        y = 0.5f * (y + (x / y));
    }
    return y;
}

static void hfi_rip_cap(float *c, float *s)
{
    const float amp = 0.20f;
    float a = (*c) * (*c) + (*s) * (*s);

    if (a > (amp * amp)) {
        const float k = amp / hfi_rip_sqrt(a);

        *c *= k;
        *s *= k;
    }
}

static void hfi_rip_decay(void)
{
    s_rip12_c *= 0.90f;
    s_rip12_s *= 0.90f;
    s_rip14_c *= 0.90f;
    s_rip14_s *= 0.90f;
}

static float hfi_ripple_iq(float iq, float w_ref)
{
    const float rpm_scale = 60.0f / (6.28318530718f * (float)M1_POLE_PAIRS);
    float wabs = w_ref;
    float w_fb;
    float on;

    if (wabs < 0.0f) {
        wabs = -wabs;
    }
    on = ((wabs >= 80.0f) && (wabs <= 130.0f) &&
          (hfi_sqwave_speed_run_active() != 0u)) ? 1.0f : 0.0f;
    if (on == 0.0f) {
        hfi_rip_decay();
    } else {
        const float ki = 0.02f;
        const float th = hfi_sqwave_get_theta_hat();
        const float p12 = hfi_rip_wrap(th * (12.0f / (float)M1_POLE_PAIRS));
        const float p14 = hfi_rip_wrap(th * (14.0f / (float)M1_POLE_PAIRS));
        float c12;
        float s12;
        float c14;
        float s14;
        float err;

        w_fb = hfi_sqwave_get_pll_int_el() * rpm_scale;
        err = w_ref - w_fb;
        motor_trig_sincos(p12, &c12, &s12);
        motor_trig_sincos(p14, &c14, &s14);
        s_rip12_c += ki * M1_SPEED_TS_S * err * c12;
        s_rip12_s += ki * M1_SPEED_TS_S * err * s12;
        s_rip14_c += ki * M1_SPEED_TS_S * err * c14;
        s_rip14_s += ki * M1_SPEED_TS_S * err * s14;
        hfi_rip_cap(&s_rip12_c, &s_rip12_s);
        hfi_rip_cap(&s_rip14_c, &s_rip14_s);
        iq += (s_rip12_c * c12) + (s_rip12_s * s12) +
              (s_rip14_c * c14) + (s_rip14_s * s14);
    }
    s_rip_amp = hfi_rip_sqrt((s_rip12_c * s_rip12_c) + (s_rip12_s * s_rip12_s)) +
                hfi_rip_sqrt((s_rip14_c * s_rip14_c) + (s_rip14_s * s_rip14_s));
    if (iq > M1_SPEED_IQ_REF_ABS_MAX) {
        iq = M1_SPEED_IQ_REF_ABS_MAX;
    } else if (iq < -M1_SPEED_IQ_REF_ABS_MAX) {
        iq = -M1_SPEED_IQ_REF_ABS_MAX;
    }
    return iq;
}
#endif

#if M1_IDENT_ENABLE && !M1_IDENT_ID_CAL_BEFORE_STEP && !M1_SPEED_IDENT_ENABLE
/** 0=尚未 ident_flow_init；第一�?JEOC �?boot（PWM/PI/ADC 零偏已就绪） */
static uint8_t s_ident_booted;
#endif

#if M1_SPEED_IDENT_ENABLE
/** 0=尚未 speed_ident boot；第一�?JEOC �?boot（与 Bode 同理，避�?init �?PLL/编码器未稳） */
static uint8_t s_speed_ident_booted;
#endif

#if M1_PLL_ENABLE
static motor_pll_t s_m1_pll;
static float s_pll_theta_mech_prev;
static uint8_t s_pll_theta_mech_prev_valid;
static float s_pll_omega_mech_rpm;
#endif

/** 20 kHz unwrap 机械�?[rad]，外�?VOFA 只读 */
static float s_theta_mech_rad;

#if (M1_HFI_GATE == 78) || (M1_HFI_GATE == 79) || (M1_HFI_GATE == 80)
#ifndef M1_HFI_VESC_WIN_HI_RPM
#define M1_HFI_VESC_WIN_HI_RPM          (1050.0f)
#endif
#ifndef M1_HFI_VESC_WIN_LO_RPM
#define M1_HFI_VESC_WIN_LO_RPM          (950.0f)
#endif
/** ≥HI 置 want_smo，≤LO 清零（VESC 式滞回）。S1 观察；S2 硬关；S2b 软交接。 */
static uint8_t s_vesc_win_smo;
#if M1_HFI_GATE == 80
static uint8_t s_vesc_ho_active;
static uint32_t s_vesc_ho_n;
#endif

static void hfi_vesc_win_obs_update(float w_rpm)
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

/** 速度/位置观测：与 outer_mode 无关，持续更新 dbg */
static void motor_current_update_observation_dbg(void)
{
    dbg.outer_theta_mech_rad = s_theta_mech_rad;
#if M1_PLL_ENABLE
    dbg.outer_omega_mech_rpm = s_speed_fb_rpm;
#endif
#if (M1_HFI_GATE == 78) || (M1_HFI_GATE == 79) || (M1_HFI_GATE == 80)
    {
        /* 诊断优先 enc PLL；过低则退回速度反馈 / HFI ω */
        float w = dbg.pll_omega_mech_rpm;

        if (w < 0.0f) {
            w = -w;
        }
        if (w < 20.0f) {
            w = s_speed_fb_rpm;
            if (w < 0.0f) {
                w = -w;
            }
        }
        if (w < 20.0f) {
            w = dbg.hfi_omega_rpm;
            if (w < 0.0f) {
                w = -w;
            }
        }
        hfi_vesc_win_obs_update(w);
    }
#endif
}

#if M1_SPEED_LOOP_ENABLE
static uint8_t s_speed_slow_div;
#endif
#if M1_IF_ENABLE && M1_IF_TO_OBS_ENABLE
static uint8_t s_if_to_obs_handed; /* 1=已进 OBS（I/F 已释放） */
static float s_if_omega_cmd_latched; /* 释放前最后一�?I/F 指令�?*/
#if M1_IF_OBS_BLEND_SPEED_ENABLE
static uint8_t s_if_blend_speed_on; /* 1=BLEND 起已开弱速度�?*/
static uint8_t s_if_blend_speed_div;
#if M1_IF_OBS_CRUISE_ENABLE
static uint8_t s_if_cruise_armed;
static float s_if_cruise_settle_s;
#endif
#endif
#if M1_IF_OBS_ANGLE_ONLY_ENABLE
static float s_if_obs_iq_freeze; /* 只切角：OBS 后钉死的 Iq */
#endif
#endif
#if M1_ENC_OPTIONAL_ENABLE || (M1_IF_ENABLE && M1_IF_TO_OBS_ENABLE)
/* 上一�?Park 角：电流重构/无感兜底，避免拔编码器后吃垃�?θ_enc */
static float s_theta_park_last;
#endif
#if M1_HFI_ENABLE && M1_HFI_MOTION_BYPASS_ENABLE
/* RUN 起用上一�?θ̂ 做扇区重构。未�?RUN 前不拿编码器角改电流�?*/
static float s_hfi_recon_theta;
static uint8_t s_hfi_recon_theta_ok;
#endif
#if M1_HFI_ENABLE && M1_HFI_MOTION_BYPASS_ENABLE && M1_EMF_SMO_ENABLE && \
    M1_EMF_PLL_ENABLE
/* 20 ms 滑动平均�?0 kHz �?400 拍。交接看这个，不看含 Kp·ε 的瞬�?ω�?*/
#define HFI_SMO_W_MA_N 400u
static float s_smo_w_hist[HFI_SMO_W_MA_N];
static float s_smo_w_sum;
static uint16_t s_smo_w_i;
static uint16_t s_smo_w_fill;

static void hfi_smo_w_ma_reset(void)
{
    s_smo_w_sum = 0.0f;
    s_smo_w_i = 0u;
    s_smo_w_fill = 0u;
}

static float hfi_smo_w_ma_step(float rpm)
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

#if M1_IF_ENABLE && M1_IF_TO_OBS_ENABLE && M1_IF_OBS_DIR_SEQ_ENABLE
/**
 * @brief 进滑行时�?SMO/PLL，避�?Iq=0 �?ω̂ 假挂�?144 ~400rpm�?
 */
static void motor_current_dir_seq_try_obs_reset(void)
{
    if (motor_outer_if_obs_dir_seq_consume_obs_reset() == 0u) {
        return;
    }
#if M1_EMF_SMO_ENABLE
    emf_smo_reset(&s_emf_smo);
#endif
#if M1_EMF_PLL_ENABLE
    emf_pll_reset(&s_emf_pll);
#endif
#if M1_OBS_SPD_PLL_ENABLE && M1_PLL_ENABLE
    motor_pll_reset(&s_obs_spd_pll, 0.0f);
    s_obs_spd_pll_primed = 0u;
    s_obs_spd_div = 0u;
    s_obs_spd_pll_rpm = 0.0f;
    s_obs_spd_fb_lpf_rpm = 0.0f;
    s_obs_spd_fb_lpf_on = 0u;
#endif
#if M1_PLL_ENABLE
    s_speed_fb_rpm = 0.0f;
#endif
}

/**
 * @brief 近零后反转再起：清观�?软切，I/F 目标�?−|ω| �?arm
 * @note 须在 cruise_tick 置位 rearm 之后调用；本�?I/F �?tick，下拍起爬坡�?
 */
static void motor_current_dir_seq_try_rearm(motor_context_t *ctx)
{
    float abs_tgt;

    if ((ctx == NULL) || (motor_outer_if_obs_dir_seq_consume_rearm() == 0u)) {
        return;
    }

    abs_tgt = M1_IF_TARGET_RPM;
    if (abs_tgt < 0.0f) {
        abs_tgt = -abs_tgt;
    }

#if M1_EMF_SMO_ENABLE
    emf_smo_reset(&s_emf_smo);
#endif
#if M1_EMF_PLL_ENABLE
    emf_pll_reset(&s_emf_pll);
#endif
#if M1_OBS_SOFT_SWITCH_ENABLE
    obs_soft_switch_reset();
#endif
#if M1_OBS_SPD_PLL_ENABLE && M1_PLL_ENABLE
    motor_pll_reset(&s_obs_spd_pll, 0.0f);
    s_obs_spd_pll_primed = 0u;
    s_obs_spd_div = 0u;
    s_obs_spd_pll_rpm = 0.0f;
    s_obs_spd_fb_lpf_rpm = 0.0f;
    s_obs_spd_fb_lpf_on = 0u;
#endif

    s_if_to_obs_handed = 0u;
#if M1_IF_OBS_BLEND_SPEED_ENABLE
    s_if_blend_speed_on = 0u;
    s_if_blend_speed_div = 0u;
#if M1_IF_OBS_CRUISE_ENABLE
    s_if_cruise_armed = 0u;
    s_if_cruise_settle_s = 0.0f;
#endif
#endif

    motor_if_set_target_rpm(-abs_tgt);
    motor_if_arm(ctx);
    ctx->iq_ref = 0.0f;
    ctx->id_ref = 0.0f;
    ctx->omega_ref = 0.0f;
    dbg.outer_omega_ref = 0.0f;
    dbg.open_seq_phase = 240u;
}
#endif

#define M1_ACDC_WINDOW_TICKS  10000u

static void motor_current_update_acdc(float id, float iq)
{
    static uint32_t tick;
    static float id_min;
    static float id_max;
    static float iq_min;
    static float iq_max;
    static float id_sum;
    static float iq_sum;
    float id_mean;
    float iq_mean;
    float id_pp;
    float iq_pp;

    if (tick == 0u) {
        id_min = id;
        id_max = id;
        iq_min = iq;
        iq_max = iq;
        id_sum = 0.0f;
        iq_sum = 0.0f;
    } else {
        if (id < id_min) {
            id_min = id;
        }
        if (id > id_max) {
            id_max = id;
        }
        if (iq < iq_min) {
            iq_min = iq;
        }
        if (iq > iq_max) {
            iq_max = iq;
        }
    }

    id_sum += id;
    iq_sum += iq;
    tick++;

    if (tick < M1_ACDC_WINDOW_TICKS) {
        return;
    }

    id_mean = id_sum / (float)M1_ACDC_WINDOW_TICKS;
    iq_mean = iq_sum / (float)M1_ACDC_WINDOW_TICKS;
    id_pp = id_max - id_min;
    iq_pp = iq_max - iq_min;

    if (id_mean >= 0.0f) {
        dbg.id_acdc = (id_mean > 0.05f) ? (id_pp / (2.0f * id_mean)) : id_pp;
    } else {
        dbg.id_acdc = (id_mean < -0.05f) ? (id_pp / (-2.0f * id_mean)) : id_pp;
    }

    if (iq_mean >= 0.0f) {
        dbg.iq_acdc = (iq_mean > 0.05f) ? (iq_pp / (2.0f * iq_mean)) : iq_pp;
    } else {
        dbg.iq_acdc = (iq_mean < -0.05f) ? (iq_pp / (-2.0f * iq_mean)) : iq_pp;
    }

    tick = 0u;
}

#if M1_CURRENT_RECON_ENABLE
static float motor_current_uq_for_sector(const motor_context_t *ctx)
{
    if (ctx->mode == M1_CTRL_CURRENT_LOOP) {
        return ctx->uq_pi;
    }
    return ctx->uq_open;
}

static float motor_current_ud_for_sector(const motor_context_t *ctx)
{
    if (ctx->mode == M1_CTRL_CURRENT_LOOP) {
        return ctx->ud_pi;
    }
    return 0.0f;
}

static void motor_current_reconstruct_abc(const motor_context_t *ctx,
                                           float theta_svpwm,
                                           float *ia, float *ib, float *ic)
{
    float Uq;
    float Ud;
    int sec;
    float a;
    float b;
    float c;

    if (ctx->mode == M1_CTRL_CURRENT_LOOP) {
        return;
    }

    Uq = motor_current_uq_for_sector(ctx);
    Ud = motor_current_ud_for_sector(ctx);
    sec = svpwm_sector_from_uq_ud(Uq, Ud, theta_svpwm);
    if (sec != 1 && sec != 2) {
        return;
    }

    a = (*ia >= 0.0f) ? *ia : -*ia;
    b = (*ib >= 0.0f) ? *ib : -*ib;
    c = (*ic >= 0.0f) ? *ic : -*ic;
    if (a < M1_CURRENT_RECON_MIN_A && b < M1_CURRENT_RECON_MIN_A &&
        c < M1_CURRENT_RECON_MIN_A) {
        return;
    }

    *ic = -(*ia + *ib);
}
#endif

void motor_current_init(bsp_axis_t *axis)
{
    if (axis == NULL) {
        return;
    }

    s_m1_ctx.pole_pairs = (uint8_t)M1_POLE_PAIRS;
    motor_open_sweep_init(&s_m1_ctx);
#if M1_SPEED_LOOP_ENABLE && (!M1_IF_ENABLE || M1_IF_TO_OBS_ENABLE)
    motor_outer_loop_init(&s_m1_ctx);
#endif
#if M1_IF_ENABLE && M1_IF_TO_OBS_ENABLE
    s_if_to_obs_handed = 0u;
    s_if_omega_cmd_latched = 0.0f;
#if M1_IF_OBS_BLEND_SPEED_ENABLE
    s_if_blend_speed_on = 0u;
    s_if_blend_speed_div = 0u;
#if M1_IF_OBS_CRUISE_ENABLE
    s_if_cruise_armed = 0u;
    s_if_cruise_settle_s = 0.0f;
#endif
#endif
#if M1_IF_OBS_ANGLE_ONLY_ENABLE
    s_if_obs_iq_freeze = M1_IF_HANDOFF_IQ_A;
#endif
#endif
#if M1_IF_ENABLE
    s_m1_ctx.mode = M1_CTRL_CURRENT_LOOP;
    s_m1_ctx.id_ref = M1_IF_ID_A;
    s_m1_ctx.iq_ref = M1_IF_IQ_A;
    dbg.open_seq_phase = 240u; /* IF bringup marker */
    motor_if_init(&s_m1_ctx);
#elif M1_HFI_ENABLE
    s_m1_ctx.mode = M1_CTRL_CURRENT_LOOP;
    s_m1_ctx.id_ref = 0.0f;
    s_m1_ctx.iq_ref = 0.0f;
    dbg.open_seq_phase = 0u; /* HFI IDLE */
    hfi_sqwave_init();
#elif M1_IDENT_ENABLE || M1_DEADBAND_FLOW_ENABLE || M1_SPEED_IDENT_ENABLE
    s_m1_ctx.mode = M1_CTRL_CURRENT_LOOP;
    s_m1_ctx.id_ref = 0.0f;
    s_m1_ctx.iq_ref = 0.0f;
#if M1_IDENT_ENABLE && !M1_IDENT_ID_CAL_BEFORE_STEP && !M1_SPEED_IDENT_ENABLE
    /* ident HOLD/Bode：推迟到第一�?JEOC。init �?TIM8 未开、ADC 零偏未完�?
     * 此时 boot 会导致上�?Id_ref 一直为 0、open_seq 对不�?60�?3�?*/
    s_ident_booted = 0u;
    dbg.open_seq_phase = 60u;
#elif M1_SPEED_IDENT_ENABLE
    /* SPEED_IDENT：推迟到第一�?JEOC（见 tick）；此处只占�?*/
    s_speed_ident_booted = 0u;
    dbg.open_seq_phase = 220u;
#elif !M1_SPEED_IDENT_ENABLE
    deadband_flow_boot(&s_m1_ctx);
#endif
#else
    dbg.open_seq_phase = 0u;
    s_m1_ctx.mode = M1_CTRL_MODE_DEFAULT;
    s_m1_ctx.id_ref = 0.0f;
#if M1_STARTUP_ENABLE
    s_m1_ctx.iq_ref = M1_STARTUP_IQ_REF_A;
#else
    s_m1_ctx.iq_ref = M1_IQ_REF_A;
#endif
    if (s_m1_ctx.mode == M1_CTRL_OBSERVE_ONLY ||
        s_m1_ctx.mode == M1_CTRL_OPEN_LOOP) {
        motor_open_sweep_arm_v012(&s_m1_ctx);
    }
#endif
    s_m1_ctx.ud_pi = 0.0f;
    s_m1_ctx.uq_pi = 0.0f;
    motor_foc_loop_pi_init(&s_m1_ctx);
#if M1_STARTUP_ENABLE
    motor_startup_init(&s_m1_ctx);
#endif
    {
        deadband_service_boot_t db_boot;

        deadband_service_boot(&db_boot);
        dbg.deadband_nvm_loaded = db_boot.nvm_loaded;
        dbg.deadband_mode = (uint8_t)deadband_service_get_mode();
    }
    axis->motor_ctx = &s_m1_ctx;

#if M1_PLL_ENABLE
    motor_pll_init(&s_m1_pll,
                   M1_PLL_KP,
                   M1_PLL_KI,
                   M1_PLL_OMEGA_LIMIT_RAD_S,
                   M1_PLL_INTEGRATOR_LIMIT_RAD_S);
    s_pll_theta_mech_prev_valid = 0u;
#endif
#if M1_EMF_VEQ_ENABLE
    emf_veq_init(&s_emf_veq);
#endif
#if M1_EMF_SMO_ENABLE
    emf_smo_init(&s_emf_smo);
#endif
#if M1_EMF_PLL_ENABLE
    emf_pll_init(&s_emf_pll);
#endif
#if M1_OBS_SOFT_SWITCH_ENABLE
    obs_soft_switch_init();
#endif
#if M1_OBS_THETA_NOTCH_ENABLE
    obs_theta_notch_init();
#endif
#if M1_OBS_SPD_PLL_ENABLE && M1_PLL_ENABLE
    {
        const float wn = 6.28318530718f * M1_OBS_SPD_PLL_FN_HZ;
        const float kp = 2.0f * M1_OBS_SPD_PLL_ZETA * wn;
        const float ki = wn * wn;
        /* 电角速度限幅 �?机械限幅 × 极对�?*/
        const float wlim = M1_PLL_OMEGA_LIMIT_RAD_S * (float)M1_POLE_PAIRS;

        motor_pll_init(&s_obs_spd_pll, kp, ki, wlim, wlim);
        s_obs_spd_pll_primed = 0u;
        s_obs_spd_div = 0u;
        s_obs_spd_pll_rpm = 0.0f;
        s_obs_spd_fb_lpf_rpm = 0.0f;
        s_obs_spd_fb_lpf_on = 0u;
    }
#endif
#if M1_PLL_ENABLE
    s_speed_fb_rpm = 0.0f;
#endif
    if (axis->enc != NULL) {
        const uint16_t enc_raw0 = encoder_get_raw(axis->enc);

        s_theta_mech_rad = encoder_get_angle(axis->enc, enc_raw0);
#if M1_PLL_ENABLE
        motor_pll_reset(&s_m1_pll, s_theta_mech_rad);
#endif
    }

#if M1_SPEED_LOOP_ENABLE && (!M1_IF_ENABLE || M1_IF_TO_OBS_ENABLE)
#if M1_SPEED_PROFILE_ENABLE
#if !M1_DEADBAND_FLOW_ONE_SHOT && !M1_SPEED_IDENT_ENABLE
    motor_speed_profile_arm(&s_m1_ctx);
    motor_outer_set_mode(&s_m1_ctx, M1_OUTER_SPEED, 0.0f, 0.0f);
#endif
#elif (M1_POS_STEP_TEST_ENABLE || M1_POS_MIT_COMBO_ENABLE) && M1_SPEED_LOOP_BOOT && \
    !M1_SPEED_IDENT_ENABLE && M1_POS_LOOP_ENABLE
#if M1_POS_MIT_COMBO_ENABLE && !M1_POS_MIT_COMBO_POS_ENABLE
    motor_pos_step_request_deferred_boot();
    motor_outer_set_mode(&s_m1_ctx, M1_OUTER_TORQUE, 0.0f, 0.0f);
#else
    motor_pos_step_request_deferred_boot();
    motor_outer_set_mode(&s_m1_ctx, M1_OUTER_POSITION, 0.0f, 0.0f);
#endif
#elif M1_SPEED_REVERSAL_TEST_ENABLE && M1_SPEED_LOOP_BOOT && !M1_SPEED_IDENT_ENABLE
    motor_speed_reversal_arm(&s_m1_ctx);
    motor_outer_set_mode(&s_m1_ctx, M1_OUTER_SPEED, 0.0f, 0.0f);
#elif M1_SPEED_LOOP_BOOT && !M1_SPEED_IDENT_ENABLE && \
    M1_POS_LOOP_ENABLE && M1_POS_LOOP_BOOT
    motor_outer_set_mode(&s_m1_ctx, M1_OUTER_POSITION, 0.0f, 0.0f);
#elif M1_SPEED_LOOP_BOOT && !M1_SPEED_IDENT_ENABLE && !M1_IF_TO_OBS_ENABLE
    s_m1_ctx.omega_ref = M1_SPEED_REF_RPM_DEFAULT;
    dbg.outer_omega_ref = M1_SPEED_REF_RPM_DEFAULT;
    motor_outer_set_mode(&s_m1_ctx, M1_OUTER_SPEED, 0.0f, 0.0f);
#endif
    s_speed_slow_div = 0u;
#endif /* M1_SPEED_LOOP_ENABLE && (!IF || IF_TO_OBS) */
}

motor_context_t *motor_current_ctx(const bsp_axis_t *axis)
{
    if (axis == NULL) {
        return NULL;
    }
    return (motor_context_t *)axis->motor_ctx;
}

void motor_current_set_mode(bsp_axis_t *axis, m1_ctrl_mode_t mode)
{
    motor_context_t *ctx = motor_current_ctx(axis);

    if (ctx == NULL) {
        return;
    }

    if (mode == M1_CTRL_CURRENT_LOOP && ctx->mode != M1_CTRL_CURRENT_LOOP) {
        motor_foc_loop_pi_reset(ctx);
#if M1_IF_ENABLE
        motor_if_arm(ctx);
#elif M1_STARTUP_ENABLE
        motor_startup_arm(ctx);
#endif
    }

    ctx->mode = mode;
}

void motor_current_set_idq_ref(bsp_axis_t *axis, float id_ref, float iq_ref)
{
    motor_context_t *ctx = motor_current_ctx(axis);

    if (ctx == NULL) {
        return;
    }

    ctx->id_ref = id_ref;
    ctx->iq_ref = iq_ref;
}

float motor_current_get_pll_omega_mech_rpm(void)
{
#if M1_PLL_ENABLE
    return s_speed_fb_rpm;
#else
    return 0.0f;
#endif
}

float motor_current_get_theta_mech_rad(void)
{
    return s_theta_mech_rad;
}

void motor_current_pll_reset_now(void)
{
#if M1_PLL_ENABLE
    motor_pll_reset(&s_m1_pll, s_theta_mech_rad);
    s_pll_omega_mech_rpm = 0.0f;
    s_speed_fb_rpm = 0.0f;
    s_pll_theta_mech_prev = s_theta_mech_rad;
    s_pll_theta_mech_prev_valid = 0u;
    dbg.pll_omega_mech_rpm = 0.0f;
    dbg.pll_omega_diff_rpm = 0.0f;
    dbg.pll_omega_err_rpm = 0.0f;
    dbg.pll_theta_err_rad = 0.0f;
    dbg.outer_omega_mech_rpm = 0.0f;
#if M1_OBS_SPD_PLL_ENABLE
    motor_pll_reset(&s_obs_spd_pll, 0.0f);
    s_obs_spd_pll_primed = 0u;
    s_obs_spd_div = 0u;
    s_obs_spd_pll_rpm = 0.0f;
    s_obs_spd_fb_lpf_rpm = 0.0f;
    s_obs_spd_fb_lpf_on = 0u;
    dbg.obs_spd_pll_rpm = 0.0f;
#endif
#endif
}

#if M1_SPEED_LOOP_ENABLE
void motor_current_outer_set_mode(bsp_axis_t *axis, m1_outer_mode_t mode)
{
    motor_context_t *ctx = motor_current_ctx(axis);

    if (ctx == NULL) {
        return;
    }

    motor_outer_set_mode(ctx, mode, dbg.foc_iq,
                         motor_current_get_pll_omega_mech_rpm());
}

void motor_current_set_omega_ref_rpm(bsp_axis_t *axis, float rpm)
{
    motor_context_t *ctx = motor_current_ctx(axis);

    if (ctx == NULL) {
        return;
    }

    ctx->omega_ref = rpm;
    dbg.outer_omega_ref = rpm;
}

void motor_current_set_theta_ref_rad(bsp_axis_t *axis, float theta_rad)
{
    motor_context_t *ctx = motor_current_ctx(axis);

    if (ctx == NULL) {
        return;
    }

    ctx->theta_ref_rad = theta_rad;
    dbg.outer_theta_ref_rad = theta_rad;
}

void motor_current_arm_position_hold(bsp_axis_t *axis)
{
    motor_context_t *ctx = motor_current_ctx(axis);

    if (ctx == NULL) {
        return;
    }

    motor_outer_arm_position_hold(ctx);
}

void motor_current_set_iq_cmd(bsp_axis_t *axis, float iq_a)
{
    motor_context_t *ctx = motor_current_ctx(axis);

    if (ctx == NULL) {
        return;
    }

    if (iq_a > M1_I_REF_ABS_MAX) {
        iq_a = M1_I_REF_ABS_MAX;
    } else if (iq_a < -M1_I_REF_ABS_MAX) {
        iq_a = -M1_I_REF_ABS_MAX;
    }

    ctx->iq_cmd = iq_a;
}
#endif

void motor_startup_arm_axis(bsp_axis_t *axis)
{
#if M1_STARTUP_ENABLE
    motor_context_t *ctx = motor_current_ctx(axis);

    if (ctx == NULL) {
        return;
    }

    motor_foc_loop_pi_reset(ctx);
    motor_startup_arm(ctx);
#else
    (void)axis;
#endif
}

#if ((M1_HFI_GATE == 38) || (M1_HFI_GATE == 53) || (M1_HFI_GATE == 54) || \
     (M1_HFI_GATE == 55) || (M1_HFI_GATE == 56) || (M1_HFI_GATE == 57) || (M1_HFI_GATE == 58) || (M1_HFI_GATE == 59) || (M1_HFI_GATE == 60) || (M1_HFI_GATE == 61) || (M1_HFI_GATE == 62) || (M1_HFI_GATE == 63) || (M1_HFI_GATE == 64) || (M1_HFI_GATE == 65) || (M1_HFI_GATE == 66) || (M1_HFI_GATE == 67) || (M1_HFI_GATE == 68) || (M1_HFI_GATE == 69) || (M1_HFI_GATE == 70) || (M1_HFI_GATE == 71) || (M1_HFI_GATE == 72) || (M1_HFI_GATE == 73) || (M1_HFI_GATE == 74) || (M1_HFI_GATE == 75) || (M1_HFI_GATE == 76) || (M1_HFI_GATE == 77) || (M1_HFI_GATE == 78) || (M1_HFI_GATE == 79) || (M1_HFI_GATE == 80) || (M1_HFI_GATE == 91) || (M1_HFI_GATE == 92) || (M1_HFI_GATE == 93)) && \
    M1_SPEED_LOOP_ENABLE
/**
 * @brief 电流环出力时，用速度环同一�?PI �?Iq，不�?iq_ref�?
 * @note ω* �?whfi �?150 ms 低通。速度环不跟电流环带宽，毛刺不进指令�?
 *       影子环没有「Iq 改变转速」的反馈，积分按 1 s 泄回维持馈流的平衡，
 *       否则大约 1 rpm 的平均差也会在几秒内打满。�? rpm 以内不进积分�?
 */
static void hfi_spd_shadow_step(float iq_hold, float w_fb_rpm)
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
    /* 平衡点：fb=ω* 时输出仍等于馈流，补�?β 随转速变化的那一�?*/
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

#if M1_HFI_SMO_HAND_ENABLE && M1_EMF_PLL_ENABLE
/*
 * 交接状态机（可组合，一变量一轮）�?
 *   55：残 Vh，不开 Id�?
 *   KILL_VH：ANG 后收 Vh→VH_END�?6/57：END=0 噪；59：微地板 PASS）�?
 *   OPEN_ID：ANG→HOLD→残地板开 Id�?8 FAIL）�?
 *   OPEN_ID+KILL：ANG→VH0→IDUP(微地�?→SMO�?
 *   61：冻 iq_ref（诊断）�?2：W_HOLD——开 Id 窗速度反馈�?VH0 �?ω，SMO 软释放�?
 *   OVERLAP：旧序（勿作主路径）�?
 */
#ifndef M1_HFI_HAND_ID_OVERLAP_ENABLE
#define M1_HFI_HAND_ID_OVERLAP_ENABLE   0
#endif
#ifndef M1_HFI_HAND_OPEN_ID_ENABLE
#define M1_HFI_HAND_OPEN_ID_ENABLE      0 /* 1：角交后在残 Vh 下开 Id */
#endif
#ifndef M1_HFI_SMO_SUB_VH_ENABLE
#define M1_HFI_SMO_SUB_VH_ENABLE        0 /* 1：SMO 用 u−u_hfi，HFI 段不停观测 */
#endif
#ifndef M1_HFI_ROTATE_PI_ENABLE
#define M1_HFI_ROTATE_PI_ENABLE         0 /* 1：Park 切 SMO 时旋 Id/Iq PI */
#endif
#ifndef M1_HFI_ID_ON_FROM_RUN_ENABLE
#define M1_HFI_ID_ON_FROM_RUN_ENABLE    0
#endif
#ifndef M1_HFI_ID_PI_OFF_ENABLE
#define M1_HFI_ID_PI_OFF_ENABLE         0
#endif
#ifndef M1_HFI_HAND_IQ_HOLD_ON_IDUP
#define M1_HFI_HAND_IQ_HOLD_ON_IDUP     0 /* 诊断用：�?iq_ref；产品路径用 W_HOLD */
#endif
#ifndef M1_HFI_HAND_W_HOLD_ON_IDUP
#define M1_HFI_HAND_W_HOLD_ON_IDUP      0 /* 1：开 Id 窗速度环吃 VH0 �?ω，SMO 软释�?*/
#endif
#ifndef M1_HFI_HAND_W_REL_N
#define M1_HFI_HAND_W_REL_N            4000u /* W_HOLD 释放�?.2 s @ 20 kHz */
#endif
#ifndef M1_HFI_HAND_W_SLEW_ENABLE
#define M1_HFI_HAND_W_SLEW_ENABLE       0 /* 1: slew-limit omega_fb */
#endif
#ifndef M1_HFI_HAND_W_SLEW_RPM_S
#define M1_HFI_HAND_W_SLEW_RPM_S        (200.0f)
#endif
#ifndef M1_HFI_HAND_W_SLEW_IDUP_ONLY
#define M1_HFI_HAND_W_SLEW_IDUP_ONLY    0 /* 1: only IDUP(+SMO_N); 0: ANG..SMO (63) */
#endif
#ifndef M1_HFI_HAND_W_SLEW_SMO_N
#define M1_HFI_HAND_W_SLEW_SMO_N       4000u /* IDUP_ONLY: continue slew 0.2 s into SMO */
#endif
#ifndef M1_HFI_HAND_KILL_VH_ENABLE
#define M1_HFI_HAND_KILL_VH_ENABLE      0
#endif
#ifndef M1_HFI_HAND_VH0_SOFT_ENABLE
#define M1_HFI_HAND_VH0_SOFT_ENABLE     0 /* 1：smoothstep �?Vh */
#endif
#ifndef M1_HFI_HAND_STOP_AFTER
#define M1_HFI_HAND_STOP_AFTER          0
#endif
#ifndef M1_HFI_HAND_VH_FLOOR
#define M1_HFI_HAND_VH_FLOOR            (0.25f) /* FADE/ANG 残注�?scale */
#endif
#ifndef M1_HFI_HAND_VH_END
#define M1_HFI_HAND_VH_END              (0.0f) /* KILL 终点 scale�?0=微地�?*/
#endif
#ifndef M1_HFI_HAND_ID_WEAK
#define M1_HFI_HAND_ID_WEAK             (0.12f)
#endif
#ifndef M1_HFI_HAND_HOLD_N
#define M1_HFI_HAND_HOLD_N             10000u /* 0.5 s：ang=1 后冻�?*/
#endif
#ifndef M1_HFI_HAND_VH0_N
#define M1_HFI_HAND_VH0_N              20000u /* 地板→END 时长 */
#endif
#ifndef M1_HFI_HAND_IDUP_N
#define M1_HFI_HAND_IDUP_N             20000u /* 1.0 s：只�?Id */
#endif
#ifndef M1_HFI_HAND_FADE_N
#define M1_HFI_HAND_FADE_N             4000u /* 1→地板时�?@ 20 kHz */
#endif
/* SMO→HFI 反向�?9 对称串行。先抬微地板→残地板，再交角、满注入、交速�?*/
#ifndef M1_HFI_HAND_REV_ENABLE
#define M1_HFI_HAND_REV_ENABLE          0
#endif
#ifndef M1_HFI_HAND_REV_WAKE_ENABLE
#define M1_HFI_HAND_REV_WAKE_ENABLE     0 /* 1：RVH �?hold �?θ̂，RQUAL 过门�?RANG */
#endif
#ifndef M1_HFI_HAND_REV_VH_WAKE
#define M1_HFI_HAND_REV_VH_WAKE         M1_HFI_HAND_VH_FLOOR /* 唤醒目标 scale�?8=1.0 */
#endif
#ifndef M1_HFI_HAND_REV_RVH_HOLD_ENABLE
#define M1_HFI_HAND_REV_RVH_HOLD_ENABLE 0 /* 1：RVH �?θ̂=SMO（hold+coast），RQUAL 再放 PLL */
#endif
#ifndef M1_HFI_HAND_REV_RESEED_N
#define M1_HFI_HAND_REV_RESEED_N       2000u /* RVH hold 时每 0.1 s �?seed */
#endif
#ifndef M1_HFI_HAND_RQUAL_X_MAX_ENABLE
#define M1_HFI_HAND_RQUAL_X_MAX_ENABLE  1 /* 0：去�?x 上界�?9�?8 上界误杀健康解调�?*/
#endif
#ifndef M1_HFI_HAND_RQUAL_X_MAX
#define M1_HFI_HAND_RQUAL_X_MAX         (0.45f)
#endif
#ifndef M1_HFI_HAND_REV_OBS_ENABLE
#define M1_HFI_HAND_REV_OBS_ENABLE      0 /* 1：RVH→ROBS 旁路观察，不交角/速（70） */
#endif
#ifndef M1_HFI_HAND_REV_VH_MIRROR_ENABLE
#define M1_HFI_HAND_REV_VH_MIRROR_ENABLE 0 /* 1：Vh 按前向 VH0+FADE 反演抬（71） */
#endif
#ifndef M1_HFI_HAND_DECEL_BRAKE_ENABLE
#define M1_HFI_HAND_DECEL_BRAKE_ENABLE  0 /* 1：SMO 减速制动向 Iq 地板（75） */
#endif
#ifndef M1_HFI_HAND_DECEL_BRAKE_IQ_A
#define M1_HFI_HAND_DECEL_BRAKE_IQ_A    (1.5f) /* 制动 |Iq|_min；按 -sign(ω) 抬，不保巡航正号 */
#endif
#ifndef M1_HFI_HAND_DECEL_BRAKE_END_RPM
#define M1_HFI_HAND_DECEL_BRAKE_END_RPM (920.0f) /* SMO ω 落到此再交回速度环 */
#endif
#ifndef M1_HFI_HAND_DECEL_BRAKE_ARM_RPM
#define M1_HFI_HAND_DECEL_BRAKE_ARM_RPM (1400.0f) /* SMO ω 先爬过再允许地板 */
#endif
#ifndef M1_HFI_HAND_DECEL_BRAKE_DROP_RPM
#define M1_HFI_HAND_DECEL_BRAKE_DROP_RPM (80.0f) /* ω* 比 SMO ω 低这么多才算减速意图 */
#endif
#ifndef M1_HFI_HAND_SPD_X_KILL_HI
#define M1_HFI_HAND_SPD_X_KILL_HI       (0.62f) /* SPD 混速 x 杀门；70=0.70 避 1530 */
#endif
#ifndef M1_HFI_HAND_REV_RPM
#define M1_HFI_HAND_REV_RPM             (1400.0f) /* 减速过线触发回 HFI */
#endif
#ifndef M1_HFI_HAND_REV_ARM_RPM
#define M1_HFI_HAND_REV_ARM_RPM         (1450.0f) /* 须先爬过再允许反�?*/
#endif
#ifndef M1_HFI_HAND_RVH_N
#define M1_HFI_HAND_RVH_N              20000u /* 1.0 s：END→FLOOR */
#endif
#ifndef M1_HFI_HAND_RQUAL_N
#define M1_HFI_HAND_RQUAL_N            8000u /* 0.4 s：θ�?锁门，对�?QUAL */
#endif
#ifndef M1_HFI_HAND_RQUAL_TIMEOUT_N
#define M1_HFI_HAND_RQUAL_TIMEOUT_N    60000u /* 3.0 s 锁不上则退�?SMO */
#endif
#ifndef M1_HFI_HAND_RANG_N
#define M1_HFI_HAND_RANG_N             20000u /* 1.0 s：ang 1�? */
#endif
#ifndef M1_HFI_HAND_RFADE_N
#define M1_HFI_HAND_RFADE_N            16000u /* FLOOR�? */
#endif
#ifndef M1_HFI_HAND_RSPD_N
#define M1_HFI_HAND_RSPD_N             40000u /* 2.0 s：alpha 1�? */
#endif
#if M1_HFI_HAND_ID_OVERLAP_ENABLE || M1_HFI_HAND_OPEN_ID_ENABLE
#define M1_HFI_HAND_ID_WITH_VH          1
#else
#define M1_HFI_HAND_ID_WITH_VH          0
#endif
#define HFI_HAND_HFI   0u
#define HFI_HAND_QUAL  1u
#define HFI_HAND_SPD   2u
#define HFI_HAND_CONF  3u
#define HFI_HAND_FADE  4u
#define HFI_HAND_ANG   5u
#define HFI_HAND_HOLD  6u /* OVERLAP：满 Park + �?Vh + �?Id */
#define HFI_HAND_VH0   7u /* 地板→END；OVERLAP �?Id=WEAK，KILL �?Id 旁路 */
#define HFI_HAND_IDUP  8u /* OVERLAP：Vh=END，Id WEAK�? */
#define HFI_HAND_SMO   9u
#define HFI_HAND_RVH   10u /* 反向：vh END→FLOOR，Park �?SMO */
#define HFI_HAND_RANG  11u /* 反向：ang 1�? */
#define HFI_HAND_RFADE 12u /* 反向：vh FLOOR�? */
#define HFI_HAND_RSPD  13u /* 反向：alpha 1�? �?HFI */
#define HFI_HAND_RQUAL 14u /* 反向：θ�?重锁门（WAKE�?*/
#define HFI_HAND_ROBS  15u /* reverse observe bypass */
#define HFI_HAND_QUAL_N  8000u  /* 0.4 s @ 20 kHz */
#define HFI_HAND_BLEND_N 40000u /* 2.0 s，只交速度 */
#define HFI_HAND_CONF_N  4000u  /* 0.2 s，速度交完后再看一�?*/
#define HFI_HAND_ANG_N   20000u /* 1.0 s */
#define HFI_HAND_ANG_OK  0.436332f /* 25° */
#define HFI_HAND_ANG_ABORT 0.523599f /* 30° */
#define HFI_HAND_ANG_KILL  0.785398f /* 45° */
#define HFI_HAND_BAD_N   400u    /* 20 ms */

/** @brief Hermite smoothstep�?�?，两端斜�?0（比线性更软）�?*/
static float hfi_hand_smoothstep(float a)
{
    if (a <= 0.0f) {
        return 0.0f;
    }
    if (a >= 1.0f) {
        return 1.0f;
    }
    return a * a * (3.0f - 2.0f * a);
}

static uint8_t s_hand_state;
static uint8_t s_hand_armed;
static uint8_t s_hand_ok;
static uint16_t s_hand_n;
static uint16_t s_hand_bad;
static float s_hand_alpha; /* 速度权重 */
static float s_hand_ang;   /* 角度权重；ANG 段仍�?VH_FLOOR */
static float s_hand_vh;
static float s_hand_th;
static float s_hand_w;
static float s_hand_dth; /* 收注入前记下的平滑角差，权重只转这个 */
#if M1_HFI_ROTATE_PI_ENABLE
static float s_rot_th_prev;
static uint8_t s_rot_th_ok;
static uint8_t s_rot_armed; /* ang<1 置位；ang 到 1 旋一次 */
#endif
#if M1_HFI_HAND_REV_ENABLE
static uint8_t s_hand_rev_arm; /* 1：已爬过 REV_ARM，允许减速反�?*/
#if M1_HFI_HAND_REV_WAKE_ENABLE
static uint16_t s_hand_rev_wait; /* RQUAL 等待计数 */
static uint8_t s_hand_rev_seeded; /* RVH 已 seed 一次 */
#if M1_HFI_HAND_REV_VH_MIRROR_ENABLE
static uint8_t s_hand_rev_vh_phase; /* 0：反演 VH0；1：反演 FADE→WAKE */
#endif
#endif
#endif
#if M1_HFI_HAND_IQ_HOLD_ON_IDUP
static float s_hand_iq_hold; /* VH0 末锁定的 iq_ref；IDUP 期间钉住 */
#endif
#if M1_HFI_HAND_DECEL_BRAKE_ENABLE
static uint8_t s_hand_brake_arm; /* SMO 内已见过高 ω* */
static uint8_t s_hand_brake_on;  /* 减速 |Iq| 地板窗 */
static float s_hand_wref_prev;   /* 上一拍 ω* */
#endif
#if M1_HFI_HAND_W_HOLD_ON_IDUP
static float s_hand_w_hold; /* VH0 �?SMO 平均转速；开 Id 窗给速度�?*/
static uint16_t s_hand_w_rel_n; /* SMO 段软释放计数 */
#endif
#if M1_HFI_HAND_W_SLEW_ENABLE
static float s_hand_w_slew;
static uint8_t s_hand_w_slew_on;
static uint16_t s_hand_w_slew_n; /* SMO 续限斜率计数 */
#endif

static float hfi_hand_abs(float x)
{
    return (x < 0.0f) ? -x : x;
}

static float hfi_hand_wrap(float dth)
{
    while (dth > 3.14159265f) {
        dth -= 6.2831853f;
    }
    while (dth < -3.14159265f) {
        dth += 6.2831853f;
    }
    return dth;
}


#if M1_HFI_ROTATE_PI_ENABLE
/**
 * @brief 把上一拍 dq 电压旋到本拍 Park，积分无扰预加载
 * @note 只用于「换 d 轴定义」那一拍，不要每拍跟 θ̂ 旋
 */
static void hfi_hand_rotate_current_pi(motor_context_t *ctx, float dth,
                                       float id, float iq)
{
    float c;
    float s;
    float ud;
    float uq;
    float ud2;
    float uq2;

    if (ctx == NULL) {
        return;
    }
    motor_trig_sincos(dth, &c, &s);
    ud = ctx->ud_pi;
    uq = ctx->uq_pi;
    ud2 = ud * c + uq * s;
    uq2 = -ud * s + uq * c;
    ctx->ud_pi = ud2;
    ctx->uq_pi = uq2;
    foc_pi_bumpless(&ctx->pi_id, ud2, ctx->id_ref, id);
    foc_pi_bumpless(&ctx->pi_iq, uq2, ctx->iq_ref, iq);
}
#endif

/** SMO 20 ms 平均转�?�?电角速度。收注入�?θ̂ 只跟这个，不跟瞬�?ω�?*/
static float hfi_hand_w_el(void)
{
    return s_hand_w * 0.104719755f * (float)M1_POLE_PAIRS;
}

static void hfi_smo_hand_idle(void)
{
    s_hand_state = HFI_HAND_HFI;
    s_hand_armed = 1u;
    s_hand_n = 0u;
    s_hand_bad = 0u;
    s_hand_alpha = 0.0f;
    s_hand_ang = 0.0f;
    s_hand_vh = 1.0f;
    s_hand_dth = 0.0f;
#if M1_HFI_ROTATE_PI_ENABLE
    s_rot_th_ok = 0u;
    s_rot_armed = 1u;
#endif
#if M1_HFI_HAND_REV_ENABLE
    s_hand_rev_arm = 0u;
#if M1_HFI_HAND_REV_WAKE_ENABLE
    s_hand_rev_wait = 0u;
    s_hand_rev_seeded = 0u;
#endif
#endif
#if M1_HFI_HAND_IQ_HOLD_ON_IDUP
    s_hand_iq_hold = 0.0f;
#endif
#if M1_HFI_HAND_DECEL_BRAKE_ENABLE
    s_hand_brake_arm = 0u;
    s_hand_brake_on = 0u;
    s_hand_wref_prev = 0.0f;
#endif
#if M1_HFI_HAND_W_HOLD_ON_IDUP
    s_hand_w_hold = 0.0f;
    s_hand_w_rel_n = 0u;
#endif
#if M1_HFI_HAND_W_SLEW_ENABLE
    s_hand_w_slew = 0.0f;
    s_hand_w_slew_on = 0u;
    s_hand_w_slew_n = 0u;
#endif
    hfi_sqwave_set_hat_hold(0u);
    hfi_sqwave_set_iq_auth_hold(0u);
    hfi_sqwave_set_inj_scale(1.0f);
    hfi_sqwave_set_id_pi_release(0u);
    hfi_sqwave_set_id_pi_soft_cmd(-1.0f);
    dbg.obs_ss_alpha = 0.0f;
    dbg.obs_ss_state = 0.0f;
#if (M1_HFI_GATE == 78) || (M1_HFI_GATE == 79) || (M1_HFI_GATE == 80)
    s_vesc_win_smo = 0u;
    dbg.hfi_vesc_win_smo = 0.0f;
#if M1_HFI_GATE == 80
    s_vesc_ho_active = 0u;
    s_vesc_ho_n = 0u;
#endif
#endif
}

/** 速度还在混、注入还在时，角或解调离开切换前的带就退回 HFI。 */
static void hfi_hand_abort(void)
{
    s_hand_state = HFI_HAND_HFI;
    s_hand_n = 0u;
    s_hand_bad = 0u;
    s_hand_alpha = 0.0f;
    s_hand_ang = 0.0f;
    s_hand_vh = 1.0f;
    s_hand_dth = 0.0f;
#if M1_HFI_ROTATE_PI_ENABLE
    s_rot_th_ok = 0u;
    s_rot_armed = 1u;
#endif
#if (M1_HFI_GATE == 78) || (M1_HFI_GATE == 79) || (M1_HFI_GATE == 80)
    s_vesc_win_smo = 0u;
    dbg.hfi_vesc_win_smo = 0.0f;
#if M1_HFI_GATE == 80
    s_vesc_ho_active = 0u;
    s_vesc_ho_n = 0u;
#endif
#endif
#if M1_HFI_HAND_REV_ENABLE
    s_hand_rev_arm = 0u;
#if M1_HFI_HAND_REV_WAKE_ENABLE
    s_hand_rev_wait = 0u;
    s_hand_rev_seeded = 0u;
#endif
#endif
#if M1_HFI_HAND_IQ_HOLD_ON_IDUP
    s_hand_iq_hold = 0.0f;
#endif
#if M1_HFI_HAND_DECEL_BRAKE_ENABLE
    s_hand_brake_arm = 0u;
    s_hand_brake_on = 0u;
    s_hand_wref_prev = 0.0f;
#endif
#if M1_HFI_HAND_W_HOLD_ON_IDUP
    s_hand_w_hold = 0.0f;
    s_hand_w_rel_n = 0u;
#endif
#if M1_HFI_HAND_W_SLEW_ENABLE
    s_hand_w_slew = 0.0f;
    s_hand_w_slew_on = 0u;
    s_hand_w_slew_n = 0u;
#endif
    s_hand_armed = 0u;
    hfi_sqwave_set_hat_hold(0u);
    hfi_sqwave_set_iq_auth_hold(0u);
    hfi_sqwave_set_id_pi_release(0u);
    hfi_sqwave_set_id_pi_soft_cmd(-1.0f);
}

/**
 * @brief 注入还开着时看角差、ε、x。持�?20 ms 或一次越出大界就退回�?
 * @param ad |θ_smo−θ̂|
 * @param dw |ω_hfi−ω_smo| [rpm]
 * @param spd_blend 1=混速段：x 只走杀门、不进温和累计（1718：x 尖到 0.5 �?0.36 温和门打回）
 * @return 1 本拍已退�?
 */
static uint8_t hfi_hand_watch(float ad, float dw, uint8_t spd_blend)
{
    float ae;
    float x;
    float x_kill_hi;
    uint8_t mild;
    uint8_t kill;

    ae = hfi_sqwave_get_eps();
    if (ae < 0.0f) {
        ae = -ae;
    }
    x = hfi_sqwave_get_x_lp();
    /* x �?900 rpm 附近中心�?0.31，尖峰可�?0.45�?.53；混速勿�?0.36 温和门�?*/
    x_kill_hi = (spd_blend != 0u) ? M1_HFI_HAND_SPD_X_KILL_HI : 0.55f;
    kill = (uint8_t)((s_hand_ok == 0u) || (ad > HFI_HAND_ANG_KILL) ||
                     (ae > 0.45f) || (x < 0.18f) || (x > x_kill_hi) ||
                     (dw > 150.0f));
    if (spd_blend != 0u) {
        mild = (uint8_t)((ad > HFI_HAND_ANG_ABORT) || (ae > 0.25f) ||
                         (dw > 80.0f));
    } else {
        mild = (uint8_t)((ad > HFI_HAND_ANG_ABORT) || (ae > 0.25f) ||
                         (x < 0.22f) || (x > 0.36f) || (dw > 80.0f));
    }
    if (kill != 0u) {
        hfi_hand_abort();
        return 1u;
    }
    if (mild != 0u) {
        if (s_hand_bad < 65535u) {
            s_hand_bad++;
        }
        if (s_hand_bad >= HFI_HAND_BAD_N) {
            hfi_hand_abort();
            return 1u;
        }
    } else {
        s_hand_bad = 0u;
    }
    return 0u;
}

/**
 * @brief ε 停写。θ�?�?20 ms 平均转速走，不用带 Kp·ε 的瞬�?ω�?
 */
static void hfi_hand_follow_smo(void)
{
    hfi_sqwave_set_hat_hold(1u);
    hfi_sqwave_set_hat_coast_el(hfi_hand_w_el());
    hfi_sqwave_set_iq_auth_hold(1u);
}

/**
 * @brief 20 kHz 交接。先交速度，注入关掉之后再交角度�?
 * @param w_hfi 上一�?HFI 全转�?[rpm]，尚未混�?SMO�?
 */
static void hfi_smo_hand_step(float w_hfi)
{
    float aw;
    float dth;
    float ad;
    float dw;
    float w_use;
    uint8_t gate;

    if (hfi_sqwave_speed_run_active() == 0u) {
        hfi_smo_hand_idle();
        return;
    }

    dth = hfi_hand_wrap(s_hand_th - hfi_sqwave_get_theta_hat());
    ad = hfi_hand_abs(dth);
    dw = hfi_hand_abs(w_hfi - s_hand_w);
    gate = (uint8_t)((s_hand_ok != 0u) && (ad < HFI_HAND_ANG_OK) && (dw < 50.0f));
    w_use = (s_hand_alpha > 0.5f) ? s_hand_w : w_hfi;
    aw = hfi_hand_abs(w_use);
    if (aw < 800.0f) {
        s_hand_armed = 1u;
    }

    switch (s_hand_state) {
    case HFI_HAND_HFI:
        s_hand_alpha = 0.0f;
        s_hand_ang = 0.0f;
        s_hand_vh = 1.0f;
        s_hand_n = 0u;
        s_hand_bad = 0u;
        hfi_sqwave_set_hat_hold(0u);
        if ((s_hand_armed != 0u) && (aw >= 900.0f) && (aw < 1100.0f) &&
            (gate != 0u)) {
            s_hand_state = HFI_HAND_QUAL;
        }
        break;
    case HFI_HAND_QUAL:
        s_hand_alpha = 0.0f;
        s_hand_ang = 0.0f;
        s_hand_vh = 1.0f;
        if ((s_hand_ok == 0u) || (ad > HFI_HAND_ANG_KILL) || (dw > 150.0f) ||
            (aw < 880.0f) || (aw >= 1100.0f)) {
            s_hand_state = HFI_HAND_HFI;
            s_hand_n = 0u;
            if (aw >= 1100.0f) {
                s_hand_armed = 0u;
            }
        } else if (gate != 0u) {
            s_hand_n++;
            if (s_hand_n >= HFI_HAND_QUAL_N) {
                s_hand_state = HFI_HAND_SPD;
                s_hand_n = 0u;
                s_hand_bad = 0u;
                s_hand_armed = 0u;
            }
        }
        break;
    case HFI_HAND_SPD:
        /* 只混速度。Park 仍是 θ̂，注入满幅�?*/
        s_hand_vh = 1.0f;
        s_hand_ang = 0.0f;
        if (hfi_hand_watch(ad, dw, 1u) != 0u) {
            break;
        }
        s_hand_n++;
        s_hand_alpha = (float)s_hand_n / (float)HFI_HAND_BLEND_N;
        if (s_hand_alpha >= 1.0f) {
            s_hand_alpha = 1.0f;
            s_hand_state = HFI_HAND_CONF;
            s_hand_n = 0u;
            s_hand_bad = 0u;
        }
        break;
    case HFI_HAND_CONF:
        /* 速度已是 SMO。角差稳住约 0.4 s 再收注入。单拍超�?25° 不清计时�?*/
        s_hand_alpha = 1.0f;
        s_hand_ang = 0.0f;
        s_hand_vh = 1.0f;
        hfi_sqwave_set_hat_hold(0u);
        hfi_sqwave_set_iq_auth_hold(0u);
        if ((s_hand_ok == 0u) || (ad > HFI_HAND_ANG_KILL) || (dw > 150.0f)) {
            s_hand_n = 0u;
        } else if (gate != 0u) {
            s_hand_n++;
            if (s_hand_n >= HFI_HAND_QUAL_N) {
                s_hand_state = HFI_HAND_FADE;
                s_hand_n = 0u;
            }
        }
        break;
    case HFI_HAND_FADE:
        /* Park 仍是 θ̂。第一拍记下平滑角差，之后 θ̂ �?20 ms 转速走�?*/
        s_hand_alpha = 1.0f;
        s_hand_ang = 0.0f;
        if (s_hand_n == 0u) {
            float th_s = emf_pll_theta_smooth(&s_emf_pll, hfi_hand_w_el());

            s_hand_dth = hfi_hand_wrap(th_s - hfi_sqwave_get_theta_hat());
        }
        hfi_hand_follow_smo();
        s_hand_n++;
        {
            float a = (float)s_hand_n / (float)M1_HFI_HAND_FADE_N;

#if M1_HFI_HAND_VH0_SOFT_ENABLE
            a = hfi_hand_smoothstep(a); /* 57：两端软，减�?1→地板「台阶感�?*/
#endif
            if (a > 1.0f) {
                a = 1.0f;
            }
            s_hand_vh = 1.0f - (1.0f - M1_HFI_HAND_VH_FLOOR) * a;
        }
        if (s_hand_n >= M1_HFI_HAND_FADE_N) {
            s_hand_vh = M1_HFI_HAND_VH_FLOOR;
            s_hand_state = HFI_HAND_ANG;
            s_hand_n = 0u;
        }
        break;
    case HFI_HAND_ANG:
        /* 残注入开着�? s �?Park �?θ̂ 转到平滑 SMO�?*/
        s_hand_alpha = 1.0f;
        s_hand_vh = M1_HFI_HAND_VH_FLOOR;
        hfi_hand_follow_smo();
        s_hand_n++;
        s_hand_ang = (float)s_hand_n / (float)HFI_HAND_ANG_N;
        if (s_hand_ang >= 1.0f) {
            s_hand_ang = 1.0f;
#if M1_HFI_HAND_ID_OVERLAP_ENABLE
            s_hand_state = HFI_HAND_HOLD;
#elif M1_HFI_HAND_KILL_VH_ENABLE
            /* 59/60：先�?Vh→END�?0 再在微地板上开 Id */
            s_hand_state = HFI_HAND_VH0;
#elif M1_HFI_HAND_OPEN_ID_ENABLE
            s_hand_state = HFI_HAND_HOLD; /* 58：残地板上开 Id */
#else
            s_hand_state = HFI_HAND_SMO; /* 55 */
#endif
            s_hand_n = 0u;
        }
        break;
#if M1_HFI_HAND_ID_OVERLAP_ENABLE || M1_HFI_HAND_OPEN_ID_ENABLE
    case HFI_HAND_HOLD:
        /* ang=1 已切满。专消化 Park 台阶�?724：同拍叠爆）�?*/
        s_hand_alpha = 1.0f;
        s_hand_ang = 1.0f;
        s_hand_vh = M1_HFI_HAND_VH_FLOOR;
        hfi_hand_follow_smo();
#if M1_HFI_HAND_STOP_AFTER == 2
        if (s_hand_n >= M1_HFI_HAND_HOLD_N) {
            break;
        }
#endif
        s_hand_n++;
        if (s_hand_n >= M1_HFI_HAND_HOLD_N) {
#if M1_HFI_HAND_STOP_AFTER == 2
            /* stay in HOLD */
#elif M1_HFI_HAND_OPEN_ID_ENABLE && !M1_HFI_HAND_KILL_VH_ENABLE
            s_hand_state = HFI_HAND_IDUP; /* 58：残地板开 Id */
            s_hand_n = 0u;
#else
            s_hand_state = HFI_HAND_VH0; /* OVERLAP：先�?Vh */
            s_hand_n = 0u;
#endif
        }
        break;
#endif
#if M1_HFI_HAND_ID_OVERLAP_ENABLE || M1_HFI_HAND_KILL_VH_ENABLE
    case HFI_HAND_VH0:
        /* FLOOR→END。本�?Id 旁路；OPEN_ID �?VH0 之后再开�?*/
        s_hand_alpha = 1.0f;
        s_hand_ang = 1.0f;
        hfi_hand_follow_smo();
#if M1_HFI_HAND_ID_OVERLAP_ENABLE && (M1_HFI_HAND_STOP_AFTER == 1)
        if (s_hand_n >= M1_HFI_HAND_VH0_N) {
            s_hand_vh = M1_HFI_HAND_VH_END;
            break;
        }
#endif
        s_hand_n++;
        {
            float a = (float)s_hand_n / (float)M1_HFI_HAND_VH0_N;

            if (a > 1.0f) {
                a = 1.0f;
            }
#if M1_HFI_HAND_VH0_SOFT_ENABLE
            a = hfi_hand_smoothstep(a);
#endif
            s_hand_vh = M1_HFI_HAND_VH_FLOOR +
                        (M1_HFI_HAND_VH_END - M1_HFI_HAND_VH_FLOOR) * a;
        }
        if (s_hand_n >= M1_HFI_HAND_VH0_N) {
            s_hand_vh = M1_HFI_HAND_VH_END;
#if M1_HFI_HAND_ID_OVERLAP_ENABLE
#if M1_HFI_HAND_STOP_AFTER == 1
            /* stay VH0 */
#else
            s_hand_state = HFI_HAND_IDUP;
            s_hand_n = 0u;
#endif
#elif M1_HFI_HAND_OPEN_ID_ENABLE
            s_hand_state = HFI_HAND_IDUP; /* 60：微地板后再开 Id */
            s_hand_n = 0u;
#else
            s_hand_state = HFI_HAND_SMO; /* 59：只收到 END */
            s_hand_n = 0u;
#endif
        }
        break;
#endif
#if M1_HFI_HAND_ID_OVERLAP_ENABLE || M1_HFI_HAND_OPEN_ID_ENABLE
    case HFI_HAND_IDUP:
        /* �?Id�?0：钉 VH_END�?8：钉 FLOOR�?*/
        s_hand_alpha = 1.0f;
        s_hand_ang = 1.0f;
#if M1_HFI_HAND_KILL_VH_ENABLE
        s_hand_vh = M1_HFI_HAND_VH_END;
#else
        s_hand_vh = M1_HFI_HAND_VH_FLOOR;
#endif
        hfi_hand_follow_smo();
        s_hand_n++;
        if (s_hand_n >= M1_HFI_HAND_IDUP_N) {
            s_hand_state = HFI_HAND_SMO;
            s_hand_n = 0u;
        }
        break;
#endif
    case HFI_HAND_SMO:
        s_hand_alpha = 1.0f;
        s_hand_ang = 1.0f;
#if M1_HFI_HAND_KILL_VH_ENABLE
        s_hand_vh = M1_HFI_HAND_VH_END;
#elif M1_HFI_HAND_ID_OVERLAP_ENABLE
        s_hand_vh = M1_HFI_HAND_VH_END;
#else
        s_hand_vh = M1_HFI_HAND_VH_FLOOR;
#endif
        hfi_hand_follow_smo();
#if M1_HFI_HAND_REV_ENABLE
        /* 先爬�?ARM，再减速过 REV 线才�?HFI（省时：不必守满高速）�?*/
        if (aw >= M1_HFI_HAND_REV_ARM_RPM) {
            s_hand_rev_arm = 1u;
        }
        if ((s_hand_rev_arm != 0u) && (aw <= M1_HFI_HAND_REV_RPM) &&
            (dbg.outer_omega_ref <= (M1_HFI_HAND_REV_RPM + 20.0f))) {
            s_hand_state = HFI_HAND_RVH;
            s_hand_n = 0u;
            s_hand_bad = 0u;
            s_hand_rev_arm = 0u;
#if M1_HFI_HAND_REV_WAKE_ENABLE
            s_hand_rev_seeded = 0u;
            s_hand_rev_wait = 0u;
#endif
#if M1_HFI_HAND_REV_VH_MIRROR_ENABLE
            s_hand_rev_vh_phase = 0u;
#endif
        }
#endif
        break;
#if M1_HFI_HAND_REV_ENABLE
    case HFI_HAND_RVH:
        /* vh：默认 END→WAKE；MIRROR=1 时按前向 VH0+FADE 反演。Park/α 仍 SMO。 */
        s_hand_alpha = 1.0f;
        s_hand_ang = 1.0f;
#if M1_HFI_HAND_REV_WAKE_ENABLE
#if M1_HFI_HAND_REV_RVH_HOLD_ENABLE
        if ((s_hand_rev_seeded == 0u) ||
            ((s_hand_n > 0u) &&
             ((s_hand_n % M1_HFI_HAND_REV_RESEED_N) == 0u))) {
            hfi_sqwave_seed_hat(s_hand_th, hfi_hand_w_el());
            s_hand_rev_seeded = 1u;
        }
        hfi_hand_follow_smo();
#else
        if (s_hand_rev_seeded == 0u) {
            hfi_sqwave_seed_hat(s_hand_th, hfi_hand_w_el());
            s_hand_rev_seeded = 1u;
        }
        hfi_sqwave_set_hat_hold(0u);
        hfi_sqwave_set_iq_auth_hold(1u);
#endif
#else
        hfi_hand_follow_smo();
#endif
        s_hand_n++;
#if M1_HFI_HAND_REV_VH_MIRROR_ENABLE
        /* 反演：先 VH0 逆（END→FLOOR），再 FADE 逆（FLOOR→WAKE），时长/smoothstep 同前向。 */
        {
            float a;
            float vh_wake = M1_HFI_HAND_REV_VH_WAKE;
            uint8_t done = 0u;

            if (vh_wake > 1.0f) {
                vh_wake = 1.0f;
            }
            if (s_hand_rev_vh_phase == 0u) {
                a = (float)s_hand_n / (float)M1_HFI_HAND_VH0_N;
                if (a > 1.0f) {
                    a = 1.0f;
                }
#if M1_HFI_HAND_VH0_SOFT_ENABLE
                a = hfi_hand_smoothstep(a);
#endif
                /* 前向 VH0：FLOOR+(END-FLOOR)*a → 反演 END+(FLOOR-END)*a */
                s_hand_vh = M1_HFI_HAND_VH_END +
                            (M1_HFI_HAND_VH_FLOOR - M1_HFI_HAND_VH_END) * a;
                if (s_hand_n >= M1_HFI_HAND_VH0_N) {
                    s_hand_vh = M1_HFI_HAND_VH_FLOOR;
                    if (vh_wake <= (M1_HFI_HAND_VH_FLOOR + 1.0e-4f)) {
                        done = 1u;
                    } else {
                        s_hand_rev_vh_phase = 1u;
                        s_hand_n = 0u;
                    }
                }
            } else {
                float span_full = 1.0f - M1_HFI_HAND_VH_FLOOR;
                float span_need = vh_wake - M1_HFI_HAND_VH_FLOOR;
                uint32_t n_fade = M1_HFI_HAND_FADE_N;

                if (span_full < 1.0e-4f) {
                    span_full = 1.0e-4f;
                }
                if (span_need < 0.0f) {
                    span_need = 0.0f;
                }
                /* 同斜率：只走 FADE 中 FLOOR→WAKE 这一段 */
                n_fade = (uint32_t)((float)M1_HFI_HAND_FADE_N * (span_need / span_full) + 0.5f);
                if (n_fade < 1u) {
                    n_fade = 1u;
                }
                a = (float)s_hand_n / (float)n_fade;
                if (a > 1.0f) {
                    a = 1.0f;
                }
#if M1_HFI_HAND_VH0_SOFT_ENABLE
                a = hfi_hand_smoothstep(a);
#endif
                /* 前向 FADE：1-(1-FLOOR)*a → 反演 FLOOR+(1-FLOOR)*a；截到 WAKE */
                s_hand_vh = M1_HFI_HAND_VH_FLOOR + span_need * a;
                if (s_hand_n >= n_fade) {
                    s_hand_vh = vh_wake;
                    done = 1u;
                }
            }
            if (done != 0u) {
#if M1_HFI_HAND_REV_WAKE_ENABLE
#if M1_HFI_HAND_REV_RVH_HOLD_ENABLE
                hfi_sqwave_seed_hat(s_hand_th, hfi_hand_w_el());
#endif
#if M1_HFI_HAND_REV_OBS_ENABLE
                s_hand_state = HFI_HAND_ROBS;
                s_hand_n = 0u;
                s_hand_bad = 0u;
#else
                s_hand_state = HFI_HAND_RQUAL;
                s_hand_n = 0u;
                s_hand_rev_wait = 0u;
                s_hand_bad = 0u;
#endif
#else
                s_hand_dth = hfi_hand_wrap(s_hand_th - hfi_sqwave_get_theta_hat());
                s_hand_state = HFI_HAND_RANG;
                s_hand_n = 0u;
#endif
            }
        }
#else
        {
            float a = (float)s_hand_n / (float)M1_HFI_HAND_RVH_N;
            float vh_wake = M1_HFI_HAND_REV_VH_WAKE;

            if (a > 1.0f) {
                a = 1.0f;
            }
#if M1_HFI_HAND_VH0_SOFT_ENABLE
            a = hfi_hand_smoothstep(a);
#endif
            if (vh_wake > 1.0f) {
                vh_wake = 1.0f;
            }
            s_hand_vh = M1_HFI_HAND_VH_END +
                        (vh_wake - M1_HFI_HAND_VH_END) * a;
        }
        if (s_hand_n >= M1_HFI_HAND_RVH_N) {
            s_hand_vh = M1_HFI_HAND_REV_VH_WAKE;
            if (s_hand_vh > 1.0f) {
                s_hand_vh = 1.0f;
            }
#if M1_HFI_HAND_REV_WAKE_ENABLE
#if M1_HFI_HAND_REV_RVH_HOLD_ENABLE
            hfi_sqwave_seed_hat(s_hand_th, hfi_hand_w_el());
#endif
#if M1_HFI_HAND_REV_OBS_ENABLE
            s_hand_state = HFI_HAND_ROBS;
            s_hand_n = 0u;
            s_hand_bad = 0u;
#else
            s_hand_state = HFI_HAND_RQUAL;
            s_hand_n = 0u;
            s_hand_rev_wait = 0u;
            s_hand_bad = 0u;
#endif
#else
            s_hand_dth = hfi_hand_wrap(s_hand_th - hfi_sqwave_get_theta_hat());
            s_hand_state = HFI_HAND_RANG;
            s_hand_n = 0u;
#endif
        }
#endif
        break;
#if M1_HFI_HAND_REV_OBS_ENABLE
    case HFI_HAND_ROBS:
        /* Park=SMO、α=SMO、vh=0.4；θ̂ 自由估，永不交角/交速。 */
        s_hand_alpha = 1.0f;
        s_hand_ang = 1.0f;
        s_hand_vh = M1_HFI_HAND_REV_VH_WAKE;
        if (s_hand_vh > 1.0f) {
            s_hand_vh = 1.0f;
        }
        hfi_sqwave_set_hat_hold(0u);
        hfi_sqwave_set_iq_auth_hold(1u);
        s_hand_n++;
        break;
#endif
#if M1_HFI_HAND_REV_WAKE_ENABLE
    case HFI_HAND_RQUAL:
        /* Park=SMO、vh=WAKE、θ�?自由跟。门过再交角；超时退�?SMO�?*/
        s_hand_alpha = 1.0f;
        s_hand_ang = 1.0f;
        s_hand_vh = M1_HFI_HAND_REV_VH_WAKE;
        if (s_hand_vh > 1.0f) {
            s_hand_vh = 1.0f;
        }
        hfi_sqwave_set_hat_hold(0u);
        hfi_sqwave_set_iq_auth_hold(1u);
        {
            float x = hfi_sqwave_get_x_lp();
            float ae = hfi_sqwave_get_eps();
            uint8_t gate_ok;

            if (ae < 0.0f) {
                ae = -ae;
            }
            /* �?解调门；勿用 dw�?506）�?9：去�?x 上界�?515 误杀）�?*/
            gate_ok = (uint8_t)((ad < HFI_HAND_ANG_OK) && (ae < 0.35f) &&
                                (x > 0.16f));
#if M1_HFI_HAND_RQUAL_X_MAX_ENABLE
            if (x >= M1_HFI_HAND_RQUAL_X_MAX) {
                gate_ok = 0u;
            }
#endif
            if (gate_ok != 0u) {
                s_hand_n++;
                s_hand_bad = 0u;
            } else {
                s_hand_n = 0u;
                if (s_hand_bad < 65535u) {
                    s_hand_bad++;
                }
            }
            if (s_hand_rev_wait < 65535u) {
                s_hand_rev_wait++;
            }
            if (s_hand_n >= M1_HFI_HAND_RQUAL_N) {
                s_hand_dth = hfi_hand_wrap(s_hand_th - hfi_sqwave_get_theta_hat());
                s_hand_state = HFI_HAND_RANG;
                s_hand_n = 0u;
                s_hand_bad = 0u;
                s_hand_rev_wait = 0u;
            } else if (s_hand_rev_wait >= M1_HFI_HAND_RQUAL_TIMEOUT_N) {
                /* 锁不上：退�?SMO 微地板，勿硬�?HFI */
                s_hand_state = HFI_HAND_SMO;
                s_hand_vh = M1_HFI_HAND_VH_END;
                s_hand_n = 0u;
                s_hand_rev_seeded = 0u;
                s_hand_rev_wait = 0u;
                hfi_hand_follow_smo();
            }
        }
        break;
#endif
    case HFI_HAND_RANG:
        /* ang 1�?。WAKE：θ�?继续跟，vh 保持 WAKE�?*/
        s_hand_alpha = 1.0f;
#if M1_HFI_HAND_REV_WAKE_ENABLE
        s_hand_vh = M1_HFI_HAND_REV_VH_WAKE;
        if (s_hand_vh > 1.0f) {
            s_hand_vh = 1.0f;
        }
        hfi_sqwave_set_hat_hold(0u);
        hfi_sqwave_set_iq_auth_hold(1u);
#else
        s_hand_vh = M1_HFI_HAND_VH_FLOOR;
        hfi_hand_follow_smo();
#endif
        s_hand_n++;
        {
            float a = (float)s_hand_n / (float)M1_HFI_HAND_RANG_N;

            if (a > 1.0f) {
                a = 1.0f;
            }
            s_hand_ang = 1.0f - a;
        }
        if (s_hand_n >= M1_HFI_HAND_RANG_N) {
            s_hand_ang = 0.0f;
#if !M1_HFI_HAND_REV_WAKE_ENABLE
            hfi_sqwave_seed_hat(s_hand_th, hfi_hand_w_el());
#endif
            hfi_sqwave_set_iq_auth_hold(0u);
            s_hand_state = HFI_HAND_RFADE;
            s_hand_n = 0u;
        }
        break;
    case HFI_HAND_RFADE:
        /* Park=θ̂，vh WAKE�?（已满则�?1）�?*/
        s_hand_alpha = 1.0f;
        s_hand_ang = 0.0f;
        hfi_sqwave_set_hat_hold(0u);
        hfi_sqwave_set_iq_auth_hold(0u);
        s_hand_n++;
        {
            float a = (float)s_hand_n / (float)M1_HFI_HAND_RFADE_N;
            float vh0 = M1_HFI_HAND_REV_VH_WAKE;

            if (a > 1.0f) {
                a = 1.0f;
            }
#if M1_HFI_HAND_VH0_SOFT_ENABLE
            a = hfi_hand_smoothstep(a);
#endif
            if (vh0 > 1.0f) {
                vh0 = 1.0f;
            }
#if !M1_HFI_HAND_REV_WAKE_ENABLE
            vh0 = M1_HFI_HAND_VH_FLOOR;
#endif
            s_hand_vh = vh0 + (1.0f - vh0) * a;
        }
        if (s_hand_n >= M1_HFI_HAND_RFADE_N) {
            s_hand_vh = 1.0f;
            s_hand_state = HFI_HAND_RSPD;
            s_hand_n = 0u;
            s_hand_bad = 0u;
        }
        break;
    case HFI_HAND_RSPD:
        /* 对称 SPD 反向：满注入，alpha 1�?（速度 SMO→HFI）�?*/
        s_hand_vh = 1.0f;
        s_hand_ang = 0.0f;
        hfi_sqwave_set_hat_hold(0u);
        hfi_sqwave_set_iq_auth_hold(0u);
        if (hfi_hand_watch(ad, dw, 1u) != 0u) {
            /* 退回失败：abort 已把状态打�?HFI */
            break;
        }
        s_hand_n++;
        {
            float a = (float)s_hand_n / (float)M1_HFI_HAND_RSPD_N;

            if (a > 1.0f) {
                a = 1.0f;
            }
            s_hand_alpha = 1.0f - a;
        }
        if (s_hand_n >= M1_HFI_HAND_RSPD_N) {
            s_hand_alpha = 0.0f;
            s_hand_state = HFI_HAND_HFI;
            s_hand_n = 0u;
            s_hand_armed = 0u; /* �?aw<800 才再允许前向 */
            s_hand_rev_arm = 0u;
        }
        break;
#endif
    default:
        s_hand_state = HFI_HAND_HFI;
        s_hand_alpha = 0.0f;
        s_hand_ang = 0.0f;
        s_hand_vh = 1.0f;
        break;
    }

    hfi_sqwave_set_inj_scale(s_hand_vh);
#if M1_HFI_HAND_ID_OVERLAP_ENABLE
    if (s_hand_state == HFI_HAND_ANG) {
        hfi_sqwave_set_id_pi_release(1u);
        hfi_sqwave_set_id_pi_soft_cmd(M1_HFI_HAND_ID_WEAK * s_hand_ang);
    } else if ((s_hand_state == HFI_HAND_HOLD) ||
               (s_hand_state == HFI_HAND_VH0)) {
        hfi_sqwave_set_id_pi_release(1u);
        hfi_sqwave_set_id_pi_soft_cmd(M1_HFI_HAND_ID_WEAK);
    } else if (s_hand_state == HFI_HAND_IDUP) {
        float a = (float)s_hand_n / (float)M1_HFI_HAND_IDUP_N;

        if (a > 1.0f) {
            a = 1.0f;
        }
        hfi_sqwave_set_id_pi_release(1u);
        hfi_sqwave_set_id_pi_soft_cmd(
            M1_HFI_HAND_ID_WEAK + (1.0f - M1_HFI_HAND_ID_WEAK) * a);
    } else if (s_hand_state == HFI_HAND_SMO) {
        hfi_sqwave_set_id_pi_release(1u);
        hfi_sqwave_set_id_pi_soft_cmd(1.0f);
    } else {
        hfi_sqwave_set_id_pi_release(0u);
        hfi_sqwave_set_id_pi_soft_cmd(-1.0f);
    }
#elif M1_HFI_HAND_OPEN_ID_ENABLE
    /* 60：VH0 旁路；IDUP soft；SMO→1。勿在 VH0 放行 Id */
    if (s_hand_state == HFI_HAND_IDUP) {
        float a = (float)s_hand_n / (float)M1_HFI_HAND_IDUP_N;

        if (a > 1.0f) {
            a = 1.0f;
        }
        a = hfi_hand_smoothstep(a);
        hfi_sqwave_set_id_pi_release(1u);
        hfi_sqwave_set_id_pi_soft_cmd(a);
    } else if (s_hand_state == HFI_HAND_SMO) {
        hfi_sqwave_set_id_pi_release(1u);
        hfi_sqwave_set_id_pi_soft_cmd(1.0f);
    } else {
        hfi_sqwave_set_id_pi_release(0u);
        hfi_sqwave_set_id_pi_soft_cmd(-1.0f);
    }
#else
#if M1_HFI_ID_ON_FROM_RUN_ENABLE || !M1_HFI_ID_PI_OFF_ENABLE
    /* 文档§7 步1：Id 环已开。KILL_VH 交接不得每拍再旁路。 */
    hfi_sqwave_set_id_pi_release(1u);
    hfi_sqwave_set_id_pi_soft_cmd(1.0f);
#else
    hfi_sqwave_set_id_pi_release(0u);
    hfi_sqwave_set_id_pi_soft_cmd(-1.0f);
#endif
#endif
#if (M1_HFI_GATE == 79) || (M1_HFI_GATE == 80)
    /* 窗：hand 内再刷（同拍贴转速）。FADE/ANG/VH0 不改。 */
    {
        float w = dbg.pll_omega_mech_rpm;

        if (w < 0.0f) {
            w = -w;
        }
        if (w < 20.0f) {
            w = s_hand_w;
            if (w < 0.0f) {
                w = -w;
            }
        }
        if (w < 20.0f) {
            w = dbg.hfi_omega_rpm;
            if (w < 0.0f) {
                w = -w;
            }
        }
        hfi_vesc_win_obs_update(w);
    }
#if M1_HFI_GATE == 79
    /* S2：want=1 且 SMO → 硬关残 Vh，Id 仍旁路（已证实会抖，仅对照）。 */
    if ((s_vesc_win_smo != 0u) && (s_hand_ok != 0u) &&
        (s_hand_state == HFI_HAND_SMO)) {
        s_hand_vh = 0.0f;
        hfi_sqwave_set_inj_scale(0.0f);
        hfi_sqwave_set_id_pi_release(0u);
        hfi_sqwave_set_id_pi_soft_cmd(-1.0f);
    }
#elif M1_HFI_GATE == 80
    /* S2b：want=1 且 SMO → VH_END→0 软收，同时放行 Id PI（soft 0→1，id* 无扰→0）。 */
    if ((s_vesc_win_smo != 0u) && (s_hand_ok != 0u) &&
        (s_hand_state == HFI_HAND_SMO)) {
        float a;

        if (s_vesc_ho_active == 0u) {
            s_vesc_ho_active = 1u;
            s_vesc_ho_n = 0u;
        }
        if (s_vesc_ho_n < M1_HFI_VESC_HANDOFF_N) {
            s_vesc_ho_n++;
        }
        a = (float)s_vesc_ho_n / (float)M1_HFI_VESC_HANDOFF_N;
        if (a > 1.0f) {
            a = 1.0f;
        }
        a = hfi_hand_smoothstep(a);
        s_hand_vh = M1_HFI_HAND_VH_END * (1.0f - a);
        hfi_sqwave_set_inj_scale(s_hand_vh);
        hfi_sqwave_set_id_pi_release(1u);
        hfi_sqwave_set_id_pi_soft_cmd(a);
    } else if (s_vesc_ho_active != 0u) {
        /* 出窗或离开 SMO：退回微地板 + Id 旁路（由上方默认分支已写 release=0） */
        s_vesc_ho_active = 0u;
        s_vesc_ho_n = 0u;
    }
#endif
#endif
    /* ch10: stage+hand；HFI→速度交→角度交（HOLD/VH0/IDUP） */
    dbg.obs_ss_alpha = 0.5f * s_hand_alpha + 0.5f * s_hand_ang;
    dbg.obs_ss_state = (float)s_hand_state;
}

#if M1_HFI_HAND_IQ_HOLD_ON_IDUP
/**
 * @brief IDUP 期间钉住 iq_ref，避免开 Id �?ω 毛刺让速度环下刹车�?
 * @note 须在 outer_tick / iq_auth 之后调用。VH0 段持续采�?hold�?
 */
static void hfi_hand_iq_hold_apply(motor_context_t *ctx)
{
    if ((ctx == NULL) || (s_hand_ok == 0u)) {
        return;
    }
    if (s_hand_state == HFI_HAND_IDUP) {
        ctx->iq_ref = s_hand_iq_hold;
        ctx->pi_speed.integrator = s_hand_iq_hold;
        dbg.outer_iq_ref = s_hand_iq_hold;
        dbg.foc_iq_ref = s_hand_iq_hold;
    } else if ((s_hand_state == HFI_HAND_VH0) ||
               (s_hand_state == HFI_HAND_ANG) ||
               (s_hand_state == HFI_HAND_HOLD)) {
        s_hand_iq_hold = ctx->iq_ref;
    }
}
#endif

#if M1_HFI_HAND_DECEL_BRAKE_ENABLE
/**
 * @brief SMO 减速段制动向 Iq 地板（产品形护角）。
 * @note 开窗：已爬高后，ω* 明显低于 SMO ω（减速意图）。守速不开。
 *       关窗：SMO ω 落到 END。速度环仍算；仅当 iq* 制动不足时抬到 -sign(ω)·floor。
 *       禁止按 |Iq| 保巡航正号（74 在开窗瞬间把 +0.5 抬到 +1.5 导致加速失步）。
 *       同步积分器到制动 iq，避免下一拍 PI 立刻顶回去。须在 iq_auth 后调用。
 */
static void hfi_hand_decel_brake_apply(motor_context_t *ctx)
{
    float wref;
    float iq;
    float fl;
    float wfb;

    if ((ctx == NULL) || (s_hand_ok == 0u)) {
        s_hand_brake_arm = 0u;
        s_hand_brake_on = 0u;
        return;
    }
    if (s_hand_state != HFI_HAND_SMO) {
        s_hand_brake_arm = 0u;
        s_hand_brake_on = 0u;
        s_hand_wref_prev = ctx->omega_ref;
        return;
    }

    wref = ctx->omega_ref;
    wfb = s_hand_w;
    if (wfb >= M1_HFI_HAND_DECEL_BRAKE_ARM_RPM) {
        s_hand_brake_arm = 1u;
    }

    if (s_hand_brake_on == 0u) {
        if ((s_hand_brake_arm != 0u) &&
            (wref < (wfb - M1_HFI_HAND_DECEL_BRAKE_DROP_RPM))) {
            s_hand_brake_on = 1u;
        }
    } else if (wfb <= M1_HFI_HAND_DECEL_BRAKE_END_RPM) {
        s_hand_brake_on = 0u;
        s_hand_brake_arm = 0u;
    }

    if (s_hand_brake_on != 0u) {
        fl = M1_HFI_HAND_DECEL_BRAKE_IQ_A;
        if (fl < 0.0f) {
            fl = -fl;
        }
        iq = ctx->iq_ref;
        /* ω>0：制动 = 负 Iq；ω<0：制动 = 正 Iq。已更负/更正则不改。 */
        if (wfb >= 0.0f) {
            if (iq > (-fl)) {
                iq = -fl;
            }
        } else if (iq < fl) {
            iq = fl;
        }
        ctx->iq_ref = iq;
        ctx->pi_speed.integrator = iq;
        dbg.outer_iq_ref = iq;
        dbg.foc_iq_ref = iq;
    }

    s_hand_wref_prev = wref;
}
#endif
#endif


#if (M1_HFI_GATE == 131) || (M1_HFI_GATE == 138) || (M1_HFI_GATE == 141)
/* 0=发布 HFI，1=发布 SMO。旧交接状态机不参与。
 * 131：门槛跟速度指令。138/141：门槛跟实测转速的绝对值。 */
static uint8_t s_hfi_pub_smo;
static float s_hfi_pub_a;
static float s_hfi_pub_theta;
static uint16_t s_hfi_pub_qual;
static uint16_t s_hfi_pub_dn;
static float s_smo_w_lp;
static uint8_t s_smo_w_init;
static uint8_t s_inj_on = 1u;

static float hfi_pub_abs(float x)
{
    return (x < 0.0f) ? -x : x;
}

static float hfi_pub_wrap(float dth)
{
    while (dth > 3.14159265f) {
        dth -= 6.2831853f;
    }
    while (dth < -3.14159265f) {
        dth += 6.2831853f;
    }
    return dth;
}

/* SMO 转速做 5 ms 低通再拿来比。差在 25° 和 50 rpm 里连续 100 ms 才交。 */
static void hfi_pub_step(float theta_smo, float omega_el, float dth)
{
    const float rpm_scale = 60.0f / (6.28318530718f * (float)M1_POLE_PAIRS);
    const float ang_ok = 0.436332f;
    const float ang_snap = 0.174533f;
#if (M1_HFI_GATE == 138) || (M1_HFI_GATE == 141)
    const float gate_rpm = hfi_pub_abs(hfi_sqwave_get_pll_int_el() * rpm_scale);
#else
    const float gate_rpm = dbg.outer_omega_ref;
#endif
    float smo_rpm;
    float dw;

    if (s_smo_w_init == 0u) {
        s_smo_w_lp = omega_el;
        s_smo_w_init = 1u;
    } else {
        s_smo_w_lp += 0.01f * (omega_el - s_smo_w_lp);
    }
    smo_rpm = s_smo_w_lp * rpm_scale;
    dw = smo_rpm - (hfi_sqwave_get_pll_int_el() * rpm_scale);
    dbg.obs_spd_rpm_err = smo_rpm;
    if (hfi_sqwave_speed_run_active() == 0u) {
        s_hfi_pub_smo = 0u;
        s_hfi_pub_a = 0.0f;
        s_hfi_pub_qual = 0u;
        s_hfi_pub_dn = 0u;
        s_inj_on = 1u;
        hfi_sqwave_set_inj_scale(1.0f);
        hfi_sqwave_set_iq_auth_hold(0u);
        dbg.obs_ss_spd_on = 0.0f;
        return;
    }
    if (s_hfi_pub_smo == 0u) {
        if ((gate_rpm >= 1300.0f) &&
            (hfi_pub_abs(dth) < ang_ok) &&
            (hfi_pub_abs(dw) < 50.0f)) {
            if (s_hfi_pub_qual < 2000u) {
                s_hfi_pub_qual++;
            }
            if (s_hfi_pub_qual >= 2000u) {
                s_hfi_pub_smo = 1u;
                s_hfi_pub_a = (hfi_pub_abs(dth) < ang_snap) ? 1.0f : 0.0f;
            }
        } else {
            s_hfi_pub_qual = 0u;
        }
    }
    if (s_hfi_pub_smo != 0u) {
        const float hat = hfi_sqwave_get_theta_hat();
        const float live = hfi_pub_wrap(theta_smo - hat);

        if (s_hfi_pub_a < 1.0f) {
            s_hfi_pub_a += (1.0f / 400.0f);
            if (s_hfi_pub_a > 1.0f) {
                s_hfi_pub_a = 1.0f;
            }
        }
        s_hfi_pub_theta = hfi_pub_wrap(hat + (s_hfi_pub_a * live));
        if (s_hfi_pub_a >= 1.0f) {
            s_hfi_pub_theta = theta_smo;
            hfi_sqwave_seed_hat(theta_smo, s_smo_w_lp);
        }
        if (gate_rpm <= 1000.0f) {
            if (s_hfi_pub_dn < 2000u) {
                s_hfi_pub_dn++;
            }
            if (s_hfi_pub_dn >= 2000u) {
                s_hfi_pub_smo = 0u;
                s_hfi_pub_a = 0.0f;
                s_hfi_pub_qual = 0u;
                s_hfi_pub_dn = 0u;
            }
        } else {
            s_hfi_pub_dn = 0u;
        }
    }
    /* 还在 HFI 上时不关注入。SMO 已发布且门槛到 1500 才关。
     * 131 的门槛是指令。138/141 的门槛是实测转速绝对值。 */
    if ((s_hfi_pub_smo != 0u) && (s_hfi_pub_a >= 1.0f) &&
        (gate_rpm >= 1500.0f)) {
        s_inj_on = 0u;
    } else if ((s_hfi_pub_smo == 0u) || (gate_rpm <= 1400.0f)) {
        s_inj_on = 1u;
    }
    hfi_sqwave_set_inj_scale((s_inj_on != 0u) ? 1.0f : 0.0f);
    hfi_sqwave_set_iq_auth_hold((s_inj_on != 0u) ? 0u : 1u);
    if (s_hfi_pub_smo == 0u) {
        dbg.obs_ss_spd_on = 0.0f;
    } else if (s_hfi_pub_a < 1.0f) {
        dbg.obs_ss_spd_on = 0.5f;
    } else if (s_inj_on != 0u) {
        dbg.obs_ss_spd_on = 1.0f;
    } else {
        dbg.obs_ss_spd_on = 2.0f;
    }
}
#endif

void motor_current_tick(bsp_axis_t *axis)
{
    motor_context_t *ctx;
    volatile uint32_t isr_t0 = *(volatile uint32_t *)&DWT->CYCCNT;
    uint16_t enc_raw;
    float i_alpha;
    float i_beta;
    float ia;
    float ib;
    float ic;
    float theta;
    float theta_enc_park;
    float theta_park;
    float sin_el;
    float cos_el;
    float id;
    float iq;
    float ud_out;
    float uq_out;
    float ua_hfi;
    float ub_hfi;
    motor_startup_step_t startup;
    volatile uint32_t foc_t0;
    volatile uint32_t t_pre_obs;
    volatile uint32_t t_post_obs;
    volatile uint32_t t_post_svpwm;

    if (axis == NULL || axis->enc == NULL || axis->pwm == NULL) {
        return;
    }

    ctx = motor_current_ctx(axis);
    if (ctx == NULL) {
        return;
    }
    ua_hfi = 0.0f;
    ub_hfi = 0.0f;

    if (axis->adc.cal_active) {
        return;
    }

    enc_raw = encoder_get_raw(axis->enc);
    dbg.enc_raw = (float)enc_raw;
    s_theta_mech_rad = encoder_get_angle(axis->enc, enc_raw);
    theta = encoder_get_theta_el(axis->enc, enc_raw, ctx->pole_pairs,
                                 encoder_get_theta_el_offset(axis->enc));
#if M1_PLL_ENABLE
    {
        const float theta_mech = s_theta_mech_rad;
        float omega_diff_rpm;

        motor_pll_update(&s_m1_pll, theta_mech, M1_CTRL_TS_S);
        s_pll_omega_mech_rpm = motor_pll_get_omega_mech_rpm(&s_m1_pll);
        dbg.pll_omega_mech_rpm = s_pll_omega_mech_rpm;
        /*
         * 速度反馈：默认编码器�?
         * OBS+速切时必须在 motor_outer_loop_tick 之前换成上一拍观测速，
         * 否则 PI 永远吃有感，拍末覆盖只影�?VOFA�?
         */
        s_speed_fb_rpm = s_pll_omega_mech_rpm;
#if M1_HFI_ENABLE && M1_HFI_MOTION_BYPASS_ENABLE
#ifndef M1_HFI_SPEED_FB_ENABLE
#define M1_HFI_SPEED_FB_ENABLE 0
#endif
#if M1_HFI_SPEED_FB_ENABLE
        /*
         * P3b：进 RUN 起速度反馈只吃上一�?HFI ω（须�?outer_tick 前）�?
         * 含踢前预锁。不回编码器 PLL，拔线后外环读到的仍�?HFI�?
         * SPEED_OBS_INT：吃积分项，不吃 Kp·eps 尖峰�?
         */
        if (hfi_sqwave_get_stage() == HFI_STAGE_RUN) {
            const float rpm_scale =
                60.0f / (2.0f * 3.14159265f * (float)M1_POLE_PAIRS);
#ifndef M1_HFI_SPEED_OBS_INT
#define M1_HFI_SPEED_OBS_INT 0
#endif
#if M1_HFI_SPEED_OBS_INT
            s_speed_fb_rpm = hfi_sqwave_get_pll_int_el() * rpm_scale;
#else
            s_speed_fb_rpm = hfi_sqwave_get_omega_el() * rpm_scale;
#endif
#if (M1_HFI_GATE == 135) || (M1_HFI_GATE == 136) || (M1_HFI_GATE == 137)
            s_speed_fb_rpm = hfi_spd_phase_rpm(s_speed_fb_rpm, ctx->omega_ref);
#elif M1_HFI_GATE == 134
            s_speed_fb_rpm = hfi_spd_lead_rpm(s_speed_fb_rpm, ctx->omega_ref);
#elif (M1_HFI_GATE == 132) || (M1_HFI_GATE == 133)
            s_speed_fb_rpm = hfi_spd_notch_rpm(s_speed_fb_rpm, ctx->omega_ref);
#elif M1_HFI_GATE == 140
            s_speed_fb_rpm = hfi_spd_notch24_rpm(s_speed_fb_rpm, ctx->omega_ref);
#endif
#if M1_HFI_SMO_HAND_ENABLE && M1_EMF_PLL_ENABLE
            hfi_smo_hand_step(s_speed_fb_rpm);
            if ((s_hand_alpha > 0.0f) && (s_hand_ok != 0u)) {
                s_speed_fb_rpm = (1.0f - s_hand_alpha) * s_speed_fb_rpm +
                                 s_hand_alpha * s_hand_w;
#if M1_HFI_HAND_W_HOLD_ON_IDUP
                /* 开 Id 是已知扰动：速度环暂用扰动前 ω，角/观测仍跟�?SMO�?*/
                if (s_hand_state == HFI_HAND_IDUP) {
                    s_speed_fb_rpm = s_hand_w_hold;
                    s_hand_w_rel_n = 0u;
                } else if ((s_hand_state == HFI_HAND_VH0) ||
                           (s_hand_state == HFI_HAND_ANG) ||
                           (s_hand_state == HFI_HAND_HOLD)) {
                    s_hand_w_hold = s_hand_w;
                    s_hand_w_rel_n = 0u;
                } else if (s_hand_state == HFI_HAND_SMO) {
                    if (s_hand_w_rel_n < M1_HFI_HAND_W_REL_N) {
                        float a = (float)s_hand_w_rel_n /
                                  (float)M1_HFI_HAND_W_REL_N;

                        a = hfi_hand_smoothstep(a);
                        s_speed_fb_rpm = (1.0f - a) * s_hand_w_hold +
                                         a * s_hand_w;
                        s_hand_w_rel_n++;
                    }
                }
#endif
#if M1_HFI_HAND_W_SLEW_ENABLE
                {
                    uint8_t slew_act = 0u;

#if M1_HFI_HAND_W_SLEW_IDUP_ONLY
                    /* 64：只盖开 Id 窗；ANG/VH0 跟活速加速，VH0 末采样作初�?*/
                    if ((s_hand_state == HFI_HAND_VH0) ||
                        (s_hand_state == HFI_HAND_ANG) ||
                        (s_hand_state == HFI_HAND_HOLD)) {
                        s_hand_w_slew = s_speed_fb_rpm;
                        s_hand_w_slew_on = 1u;
                        s_hand_w_slew_n = 0u;
                    } else if (s_hand_state == HFI_HAND_IDUP) {
                        slew_act = 1u;
                        s_hand_w_slew_n = 0u;
                    } else if (s_hand_state == HFI_HAND_SMO) {
                        if (s_hand_w_slew_n < M1_HFI_HAND_W_SLEW_SMO_N) {
                            slew_act = 1u;
                            s_hand_w_slew_n++;
                        } else {
                            s_hand_w_slew_on = 0u;
                        }
                    } else {
                        s_hand_w_slew_on = 0u;
                        s_hand_w_slew_n = 0u;
                    }
#else
                    /* 63：ANG 起全程限斜率（已证实挡加速有害） */
                    if ((s_hand_state == HFI_HAND_ANG) ||
                        (s_hand_state == HFI_HAND_HOLD) ||
                        (s_hand_state == HFI_HAND_VH0) ||
                        (s_hand_state == HFI_HAND_IDUP) ||
                        (s_hand_state == HFI_HAND_SMO)) {
                        slew_act = 1u;
                    } else {
                        s_hand_w_slew_on = 0u;
                    }
#endif
                    if (slew_act != 0u) {
                        if (s_hand_w_slew_on == 0u) {
                            s_hand_w_slew = s_speed_fb_rpm;
                            s_hand_w_slew_on = 1u;
                        } else {
                            const float step =
                                M1_HFI_HAND_W_SLEW_RPM_S * M1_CTRL_TS_S;
                            float d = s_speed_fb_rpm - s_hand_w_slew;

                            if (d > step) {
                                s_hand_w_slew += step;
                            } else if (d < -step) {
                                s_hand_w_slew -= step;
                            } else {
                                s_hand_w_slew = s_speed_fb_rpm;
                            }
                        }
                        s_speed_fb_rpm = s_hand_w_slew;
                    }
                }
#endif
            }
#endif
        }
#if M1_HFI_SMO_HAND_ENABLE && M1_EMF_PLL_ENABLE
        else {
            hfi_smo_hand_idle();
        }
#endif
#endif
#endif
#if M1_OBS_SOFT_SWITCH_ENABLE && M1_OBS_SS_SPEED_SWITCH_ENABLE && M1_OBS_SPD_PLL_ENABLE
        /* BLEND 起与角同步往观测速靠，避�?OBS 瞬间 enc→obs 硬切�?049 晃速） */
        if (obs_soft_switch_speed_use_obs() != 0u) {
            float omega_obs = s_obs_spd_pll_rpm;

#if M1_OBS_SPD_FB_LPF_HZ > 0
            if (s_obs_spd_fb_lpf_on == 0u) {
#if M1_ENC_OPTIONAL_ENABLE
                /* 无感可选：LPF 初值用 ω̂，勿用死编码�?PLL */
                s_obs_spd_fb_lpf_rpm = s_obs_spd_pll_rpm;
#else
                s_obs_spd_fb_lpf_rpm = s_pll_omega_mech_rpm;
#endif
                s_obs_spd_fb_lpf_on = 1u;
            } else {
                const float a = 6.28318530718f * (float)M1_OBS_SPD_FB_LPF_HZ *
                                M1_CTRL_TS_S;
                float alpha = (a > 1.0f) ? 1.0f : a;

                s_obs_spd_fb_lpf_rpm +=
                    alpha * (s_obs_spd_pll_rpm - s_obs_spd_fb_lpf_rpm);
            }
            omega_obs = s_obs_spd_fb_lpf_rpm;
#endif
            if (obs_soft_switch_get_state() == OBS_SS_BLEND) {
#if M1_ENC_OPTIONAL_ENABLE
                /* 可选编码器：融合段也不�?enc PLL */
                s_speed_fb_rpm = omega_obs;
#else
                const float blend = obs_soft_switch_get_alpha();
                float b = blend;

                if (b < 0.0f) {
                    b = 0.0f;
                } else if (b > 1.0f) {
                    b = 1.0f;
                }
                s_speed_fb_rpm = s_pll_omega_mech_rpm * (1.0f - b) +
                                 omega_obs * b;
#endif
            } else {
                s_speed_fb_rpm = omega_obs;
            }
        } else {
#if M1_OBS_SPD_FB_LPF_HZ > 0
            s_obs_spd_fb_lpf_on = 0u;
#endif
        }
#endif
        dbg.pll_theta_err_rad = motor_pll_get_last_err(&s_m1_pll);

        if (s_pll_theta_mech_prev_valid) {
            omega_diff_rpm = (theta_mech - s_pll_theta_mech_prev) / M1_CTRL_TS_S;
            omega_diff_rpm = omega_diff_rpm * 60.0f / 6.28318530718f;
        } else {
            omega_diff_rpm = 0.0f;
            s_pll_theta_mech_prev_valid = 1u;
        }
        dbg.pll_omega_diff_rpm = omega_diff_rpm;
        dbg.pll_omega_err_rpm =
            dbg.pll_omega_mech_rpm - dbg.pll_omega_diff_rpm;
        s_pll_theta_mech_prev = theta_mech;

#if M1_PLL_THETA_PARK_ENABLE
        theta = motor_pll_theta_el(&s_m1_pll, ctx->pole_pairs,
                                   encoder_get_theta_el_offset(axis->enc));
#endif
    }
#if M1_SPEED_IDENT_ENABLE
    /* HOLD settle：每拍清 PLL，防止编码器毛刺在开环前�?ω 顶满限幅 */
    if (speed_ident_flow_is_armed() &&
        speed_ident_module_hold_iq_inhibit()) {
        motor_current_pll_reset_now();
    }
#endif
#endif
    motor_current_update_observation_dbg();
#if M1_THETA_NEGATE
    theta_enc_park = -theta;
#else
    theta_enc_park = theta;
#endif
    dbg.enc_theta_el = theta_enc_park;

    {
        float i_phys[3];

        adc_sample_get_abc(&axis->adc, &i_phys[0], &i_phys[1], &i_phys[2]);
        motor_phase_binding_map_abc(i_phys, &ia, &ib, &ic);
#if M1_CURRENT_RECON_ENABLE
#if M1_ENC_OPTIONAL_ENABLE || (M1_IF_ENABLE && M1_IF_TO_OBS_ENABLE)
        /* 无感/可选编码器：重构用上一拍控制角，勿绑死 θ_enc */
        motor_current_reconstruct_abc(ctx, s_theta_park_last, &ia, &ib, &ic);
#elif M1_HFI_ENABLE && M1_HFI_MOTION_BYPASS_ENABLE
        if (s_hfi_recon_theta_ok != 0u) {
            motor_current_reconstruct_abc(ctx, s_hfi_recon_theta, &ia, &ib, &ic);
        }
#else
        motor_current_reconstruct_abc(ctx, theta_enc_park, &ia, &ib, &ic);
#endif
#endif
    }
    dbg.foc_ia = ia;
    dbg.foc_ib = ib;
    dbg.foc_ic = ic;
    foc_t0 = *(volatile uint32_t *)&DWT->CYCCNT;
    Clarke_Transform(ia, ib, ic, &i_alpha, &i_beta);

#if M1_IDENT_ENABLE && !M1_IDENT_ID_CAL_BEFORE_STEP && !M1_SPEED_IDENT_ENABLE
    if (s_ident_booted == 0u) {
        ident_flow_init(ctx);
        s_ident_booted = 1u;
    }
    ident_flow_tick(ctx);
#elif M1_SPEED_IDENT_ENABLE
    if (s_speed_ident_booted == 0u) {
        /* �?Bode 同：PWM/ADC/编码�?SPI 已跑后再 arm，并�?PLL 积分 */
        motor_current_pll_reset_now();
        deadband_flow_boot(ctx);
        speed_ident_flow_init(ctx);
#if M1_EMF_VEQ_ENABLE
        emf_veq_reset(&s_emf_veq);
#endif
#if M1_EMF_SMO_ENABLE
        emf_smo_reset(&s_emf_smo);
#endif
#if M1_EMF_PLL_ENABLE
        emf_pll_reset(&s_emf_pll);
#endif
#if M1_OBS_SOFT_SWITCH_ENABLE
        obs_soft_switch_reset();
#endif
#if M1_OBS_SPD_PLL_ENABLE && M1_PLL_ENABLE
        motor_pll_reset(&s_obs_spd_pll, 0.0f);
        s_obs_spd_pll_primed = 0u;
        s_obs_spd_div = 0u;
        s_obs_spd_pll_rpm = 0.0f;
        s_obs_spd_fb_lpf_rpm = 0.0f;
        s_obs_spd_fb_lpf_on = 0u;
#endif
        s_speed_ident_booted = 1u;
    }
    deadband_flow_tick(ctx);
#elif M1_IDENT_ENABLE || M1_DEADBAND_FLOW_ENABLE
    deadband_flow_tick(ctx);
#endif

#if M1_SPEED_LOOP_ENABLE
    /* 注意：s_if_to_obs_handed 是运行时变量，不能写�?#if（预处理当成 0�?
     * 会把 I/F→OBS 的速度环整段编译掉 �?iq_ref 钉死在交接值，Uq 顶满）�?*/
    {
        uint8_t run_speed_loop = 1u;

#if M1_IF_ENABLE
#if M1_IF_TO_OBS_ENABLE
        run_speed_loop = s_if_to_obs_handed;
#if M1_IF_OBS_ANGLE_ONLY_ENABLE
        /* 只切角验收：绝不跑速度环，Iq 由冻结值驱�?*/
        run_speed_loop = 0u;
#endif
#if M1_IF_OBS_DIR_SEQ_ENABLE
        /* 滑行段：松手，勿�?PI �?−Iq 当刹�?*/
        if (motor_outer_if_obs_dir_seq_is_coast() != 0u) {
            run_speed_loop = 0u;
            ctx->iq_ref = 0.0f;
            ctx->id_ref = 0.0f;
            ctx->omega_ref = 0.0f;
            dbg.outer_omega_ref = 0.0f;
        }
#endif
#else
        run_speed_loop = 0u;
#endif
#endif
        if (run_speed_loop != 0u) {
            if (++s_speed_slow_div >= M1_SPEED_DECIM) {
                s_speed_slow_div = 0u;
                motor_outer_loop_tick(ctx);
#if M1_HFI_GATE == 133
                ctx->iq_ref = hfi_ripple_iq(ctx->iq_ref, ctx->omega_ref);
#endif
            }
        }
    }
#endif

    if (ctx->mode == M1_CTRL_CURRENT_LOOP) {
#if M1_IDENT_ENABLE || M1_ID_LOCK_CAL_SWEEP
        motor_foc_loop_on_flow_tick(ctx);
#endif
#if M1_IF_ENABLE
        {
            motor_if_step_t if_step = motor_if_tick(ctx, theta_enc_park);

            if (motor_if_is_driving() != 0u) {
                theta_park = if_step.theta_park;
                ctx->id_ref = if_step.id_ref;
#if M1_IF_TO_OBS_ENABLE && M1_IF_OBS_BLEND_SPEED_ENABLE
                /* BLEND 弱速度环已接管 Iq：I/F 只供 θ，勿每拍盖回 4A�?233�?*/
                if (s_if_blend_speed_on == 0u) {
                    ctx->iq_ref = if_step.iq_ref;
                }
#else
                ctx->iq_ref = if_step.iq_ref;
#endif
#if M1_IF_TO_OBS_ENABLE
                /* 软切 gates �?omega_ref；I/F 段用指令转�?*/
                ctx->omega_ref = if_step.omega_cmd_rpm;
                dbg.outer_omega_ref = if_step.omega_cmd_rpm;
                s_if_omega_cmd_latched = if_step.omega_cmd_rpm;
#endif
                startup.theta_park = if_step.theta_park;
                startup.iq_ref = ctx->iq_ref;
                startup.omega_mech_rpm = if_step.omega_meas_rpm;
                startup.use_fixed_uq = 0u;
                startup.uq_out = 0.0f;
                startup.pi_reset = 0u;
                startup.pi_bumpless = 0u;
                startup.uq_prev = 0.0f;
                startup.ud_prev = 0.0f;
                startup.state = M1_STARTUP_CLOSED;
            } else {
#if M1_IF_TO_OBS_ENABLE
                /* 已交给无感：θ 由软切给出，Iq 由速度�?*/
#if M1_ENC_OPTIONAL_ENABLE
                theta_park = s_theta_park_last;
#else
                theta_park = theta_enc_park;
#endif
                startup.use_fixed_uq = 0u;
                startup.iq_ref = ctx->iq_ref;
                startup.pi_bumpless = 0u;
                startup.state = M1_STARTUP_CLOSED;
#else
                theta_park = theta_enc_park;
                ctx->iq_ref = 0.0f;
                startup.iq_ref = 0.0f;
                startup.use_fixed_uq = 0u;
                startup.state = M1_STARTUP_CLOSED;
#endif
            }
            dbg.startup_state = (uint8_t)if_step.state;
            dbg.startup_omega_mech_rpm = if_step.omega_meas_rpm;
            dbg.if_omega_cmd_rpm = if_step.omega_cmd_rpm;
            dbg.if_theta_err_rad = if_step.theta_err_rad;
        }
#else
        startup = motor_startup_tick(ctx, theta_enc_park);
#if M1_IDENT_ENABLE
#if M1_IDENT_FIX_THETA_ENABLE
        theta_park = M1_IDENT_THETA_EL_RAD;
#else
        theta_park = theta_enc_park;
#endif
#elif M1_ID_LOCK_CAL_SWEEP && M1_ID_CAL_FIX_THETA_ENABLE
        if (deadband_id_cal_use_fix_theta() ||
            (deadband_flow_id_cal_active() && !deadband_id_cal_in_iq_probe())) {
            theta_park = deadband_id_cal_target_theta();
        } else {
            theta_park = theta_enc_park;
        }
#else
        theta_park = startup.theta_park;
#endif
        dbg.startup_state = (uint8_t)startup.state;
        dbg.startup_omega_mech_rpm = startup.omega_mech_rpm;
        dbg.if_omega_cmd_rpm = 0.0f;
        dbg.if_theta_err_rad = 0.0f;
#endif /* M1_IF_ENABLE */
    } else {
        theta_park = theta_enc_park;
        dbg.startup_state = (uint8_t)M1_STARTUP_CLOSED;
        dbg.startup_omega_mech_rpm = 0.0f;
        dbg.if_omega_cmd_rpm = 0.0f;
        dbg.if_theta_err_rad = 0.0f;
        startup.iq_ref = 0.0f;
        startup.state = M1_STARTUP_CLOSED;
        startup.use_fixed_uq = 0u;
        startup.uq_out = 0.0f;
        startup.pi_bumpless = 0u;
        startup.uq_prev = 0.0f;
        startup.ud_prev = 0.0f;
#if M1_OPEN_UQ_PRE_ID_CAL_ENABLE || M1_OPEN_UD_PRE_ID_CAL_ENABLE
        if ((ctx->mode == M1_CTRL_OBSERVE_ONLY || ctx->mode == M1_CTRL_OPEN_LOOP) &&
            motor_open_sweep_use_fix_theta()) {
            theta_park = M1_OPEN_PRE_ID_LADDER_THETA_EL_RAD;
        }
#endif
    }
#if M1_OBS_SOFT_SWITCH_ENABLE && M1_EMF_PLL_ENABLE
    /* 用上一�?PLL θ̂；本拍观测器�?Park/SVPWM 之后更新 */
    if (s_emf_pll.primed != 0u) {
        float omega_enc_ss = 0.0f;
        float omega_obs_ss = s_obs_spd_pll_rpm; /* 上一拍观测速，�?OBS 掉速门�?*/
        float theta_ss_base = theta_enc_park;

#if M1_PLL_ENABLE
#if M1_ENC_OPTIONAL_ENABLE
        /* 可选编码器：软切不�?ω_enc；有感监督只�?VOFA */
        omega_enc_ss = 0.0f;
#else
        omega_enc_ss = s_pll_omega_mech_rpm;
#endif
#endif
#if M1_IF_ENABLE && M1_IF_TO_OBS_ENABLE
        /* I/F→OBS：融合底角用 θ_if，避免切入跳到编码器�?*/
        if (motor_if_is_driving() != 0u) {
            theta_ss_base = theta_park;
        }
#if M1_ENC_OPTIONAL_ENABLE
        else {
            /* I/F 已释放：底角用上一拍控制角，勿回落垃圾 θ_enc */
            theta_ss_base = s_theta_park_last;
        }
#endif
#endif
        {
            float theta_hat_park = s_emf_pll.theta_hat;
            float ss_theta_err = s_emf_pll.theta_err;

#if M1_OBS_THETA_NOTCH_ENABLE
            theta_hat_park = obs_theta_notch_apply(s_emf_pll.theta_hat,
                                                   omega_obs_ss,
                                                   obs_soft_switch_speed_on_obs(),
                                                   M1_CTRL_TS_S);
#endif
#if M1_IF_ENABLE && M1_IF_TO_OBS_ENABLE
            /* 门限角差�?θ̂−θ_if，不依赖编码器（拔掉编码器仍可判�?*/
            if (motor_if_is_driving() != 0u) {
                float e = theta_hat_park - theta_ss_base;

                while (e > 3.14159265359f) {
                    e -= 6.28318530718f;
                }
                while (e < -3.14159265359f) {
                    e += 6.28318530718f;
                }
                ss_theta_err = e;
            }
#endif
            theta_park = obs_soft_switch_apply(theta_ss_base,
                                               theta_hat_park,
                                               ss_theta_err,
                                               s_emf_pll.emag,
                                               omega_enc_ss,
                                               omega_obs_ss,
                                               ctx->omega_ref,
                                               dbg.foc_iq,
                                               M1_CTRL_TS_S);
        }
#if M1_IF_ENABLE && M1_IF_TO_OBS_ENABLE
        /* 进入 OBS：释�?I/F。ANGLE_ONLY=固定 Iq；否则速度�?bumpless 接管 */
        if ((s_if_to_obs_handed == 0u) &&
            (obs_soft_switch_speed_on_obs() != 0u)) {
            const float omega_meas = s_speed_fb_rpm;
            float omega_hold;

            s_if_to_obs_handed = 1u;
            motor_if_release();

            /* HOLD：优先钉 I/F 指令速，避免�?BLEND 飞车速当成目标（2249�?*/
#if M1_IF_OBS_HOLD_SPEED_ENABLE && M1_IF_OBS_HOLD_IF_CMD_ENABLE
            omega_hold = s_if_omega_cmd_latched;
            /* �?|ω| 判有效：负速时 latched<1 不能当“未就绪�?*/
            if ((omega_hold > -1.0f) && (omega_hold < 1.0f)) {
                omega_hold = omega_meas;
            }
#elif M1_IF_OBS_HOLD_SPEED_ENABLE
            omega_hold = omega_meas;
#else
            omega_hold = M1_IF_OBS_SPEED_REF_RPM;
#endif
            ctx->omega_ref = omega_hold;
            dbg.outer_omega_ref = omega_hold;
#if M1_IF_OBS_ANGLE_ONLY_ENABLE
            /* 只切角：�?Iq，不启动速度外环 */
            s_if_obs_iq_freeze = M1_IF_HANDOFF_IQ_A;
            ctx->iq_ref = s_if_obs_iq_freeze;
            ctx->id_ref = M1_IF_ID_A;
            dbg.open_seq_phase = 242u; /* IF→OBS angle-only */
#else
#if M1_IF_OBS_BLEND_SPEED_ENABLE
            if (s_if_blend_speed_on != 0u) {
                /* BLEND 已开外环：延续当�?iq_ref，只�?ω_IF */
                motor_outer_sync_speed_boot(ctx, ctx->iq_ref, omega_meas);
                motor_outer_set_omega_ramp_rpm(omega_hold);
                ctx->omega_ref = omega_hold;
                dbg.outer_omega_ref = omega_hold;
                dbg.open_seq_phase = 245u; /* IF→OBS, speed from BLEND */
            } else
#endif
            {
                const float iq_boot = M1_IF_OBS_HANDOFF_IQ_BOOT_A;

                ctx->iq_ref = iq_boot;
                /* bumpless：ref=ω_hold，fb=实测；斜坡起点钉�?hold */
                motor_outer_set_mode(ctx, M1_OUTER_SPEED, iq_boot, omega_meas);
                motor_outer_sync_speed_boot(ctx, iq_boot, omega_meas);
                motor_outer_set_omega_ramp_rpm(omega_hold);
                ctx->omega_ref = omega_hold;
                dbg.outer_omega_ref = omega_hold;
#if M1_IF_OBS_EW_CLAMP_ENABLE
                motor_outer_if_obs_ew_guard_arm();
#endif
#if M1_IF_OBS_SOFT_BRAKE_ENABLE
                motor_outer_if_obs_soft_brake_arm(ctx);
                /* 浅刹后再�?bumpless：防 slew 仍停�?I/F �?+Iq */
                motor_outer_sync_speed_boot(ctx, ctx->iq_ref, omega_meas);
#endif
                dbg.open_seq_phase = 243u; /* IF→OBS handed, ω_ref=ω_IF */
            }
#endif
        }
#if M1_IF_OBS_CRUISE_ENABLE && M1_IF_TO_OBS_ENABLE
        /* 速切站稳 �?三步巡航（①ref ②Iq ③PI�?*/
        if (obs_soft_switch_speed_use_obs() != 0u) {
            if (s_if_cruise_armed == 0u) {
                s_if_cruise_settle_s += M1_CTRL_TS_S;
                if (s_if_cruise_settle_s >= M1_IF_OBS_CRUISE_SETTLE_S) {
                    motor_outer_if_obs_cruise_arm(ctx, s_speed_fb_rpm);
                    s_if_cruise_armed = 1u;
                }
            } else {
                motor_current_dir_seq_try_obs_reset();
                motor_outer_if_obs_cruise_tick(ctx, s_speed_fb_rpm, M1_CTRL_TS_S);
            }
#if M1_IF_OBS_DIR_SEQ_ENABLE
            motor_current_dir_seq_try_rearm(ctx);
#endif
        } else {
            s_if_cruise_settle_s = 0.0f;
        }
#endif
#if M1_IF_OBS_ANGLE_ONLY_ENABLE
        /* 已切角：每拍钉死 Iq，防止其它路径改�?*/
        if (s_if_to_obs_handed != 0u) {
            ctx->iq_ref = s_if_obs_iq_freeze;
            ctx->id_ref = M1_IF_ID_A;
        }
#endif
        /* BLEND：KEEP_IF_IQ 强制拖动电流；BLEND_SPEED 则本段外环写 Iq */
        if ((motor_if_is_driving() != 0u) &&
            (obs_soft_switch_get_state() == OBS_SS_BLEND)) {
#if M1_IF_OBS_BLEND_SPEED_ENABLE && !M1_IF_OBS_ANGLE_ONLY_ENABLE
            {
                const float omega_meas_b = s_speed_fb_rpm;
                float omega_hold_b = s_if_omega_cmd_latched;

                /* �?OBS 交接：|ω|<1 才回退实测（兼容反向） */
                if ((omega_hold_b > -1.0f) && (omega_hold_b < 1.0f)) {
                    omega_hold_b = omega_meas_b;
                }
                if (s_if_blend_speed_on == 0u) {
                    /* 转子系：反向驱动 −Iq；勿�?I/F �?+Iq 否则浅刹�?max→失�?*/
                    const float iq_boot = M1_IF_OBS_SPEED_IQ_BOOT_A;

                    s_if_blend_speed_on = 1u;
                    s_if_blend_speed_div = 0u;
                    ctx->omega_ref = omega_hold_b;
                    dbg.outer_omega_ref = omega_hold_b;
                    ctx->iq_ref = iq_boot;
                    ctx->id_ref = M1_IF_ID_A;
                    motor_outer_set_mode(ctx, M1_OUTER_SPEED, iq_boot, omega_meas_b);
                    motor_outer_sync_speed_boot(ctx, iq_boot, omega_meas_b);
                    motor_outer_set_omega_ramp_rpm(omega_hold_b);
#if M1_IF_OBS_EW_CLAMP_ENABLE
                    motor_outer_if_obs_ew_guard_arm();
#endif
#if M1_IF_OBS_SOFT_BRAKE_ENABLE
                    motor_outer_if_obs_soft_brake_arm(ctx);
                    motor_outer_sync_speed_boot(ctx, ctx->iq_ref, omega_meas_b);
#endif
                    dbg.open_seq_phase = 244u; /* BLEND + weak speed */
                }
                ctx->omega_ref = omega_hold_b;
                dbg.outer_omega_ref = omega_hold_b;
                motor_outer_set_omega_ramp_rpm(omega_hold_b);
                ctx->id_ref = M1_IF_ID_A;
                if (++s_if_blend_speed_div >= M1_SPEED_DECIM) {
                    s_if_blend_speed_div = 0u;
                    motor_outer_loop_tick(ctx);
                }
            }
#elif M1_IF_OBS_ANGLE_ONLY_ENABLE || M1_IF_OBS_BLEND_KEEP_IF_IQ
            ctx->iq_ref = M1_IF_IQ_A;
            ctx->id_ref = M1_IF_ID_A;
#else
            const float a = obs_soft_switch_get_alpha();
            float a_clamped = a;

            if (a_clamped < 0.0f) {
                a_clamped = 0.0f;
            } else if (a_clamped > 1.0f) {
                a_clamped = 1.0f;
            }
            ctx->iq_ref = M1_IF_IQ_A +
                (M1_IF_HANDOFF_IQ_A - M1_IF_IQ_A) * a_clamped;
            ctx->id_ref = M1_IF_ID_A;
#endif
        }
#endif
    } else {
#if !(M1_IF_ENABLE && M1_IF_TO_OBS_ENABLE)
        theta_park = theta_enc_park;
#else
        if (motor_if_is_driving() == 0u) {
#if M1_ENC_OPTIONAL_ENABLE
            theta_park = s_theta_park_last;
#else
            theta_park = theta_enc_park;
#endif
        }
        /* else：保�?I/F θ，等 PLL primed */
#endif
    }
    dbg.obs_ss_state = (float)obs_soft_switch_get_state();
    dbg.obs_ss_alpha = obs_soft_switch_get_alpha();
    dbg.obs_ss_spd_on = (float)obs_soft_switch_speed_use_obs();
#endif
#if M1_HFI_ENABLE
    hfi_sqwave_on_angle(theta_enc_park, M1_CTRL_TS_S);
    theta_park = hfi_sqwave_park_theta(theta_enc_park);
#if (M1_HFI_GATE == 131) || (M1_HFI_GATE == 138) || (M1_HFI_GATE == 141)
    if (s_hfi_pub_smo != 0u) {
        theta_park = s_hfi_pub_theta;
    }
#endif
#if M1_HFI_GATE == 132
    dbg.obs_ss_spd_on = (s_spd_notch_on != 0u) ? 1.0f : 0.0f;
#elif M1_HFI_GATE == 133
    dbg.obs_ss_spd_on = s_rip_amp;
#elif M1_HFI_GATE == 134
    dbg.obs_ss_spd_on = s_lead_rep;
#elif (M1_HFI_GATE == 135) || (M1_HFI_GATE == 136) || (M1_HFI_GATE == 137)
    dbg.obs_ss_spd_on = s_ph_rep;
#endif
#if M1_HFI_SMO_HAND_ENABLE && M1_EMF_PLL_ENABLE
    /* 注入开着时不�?Park。角度权重只在注入已收掉之后�?0�?*/
    hfi_sqwave_set_torque_theta(0.0f, 0u);
    if (s_hand_ang >= 1.0f) {
        /* 交完：Park 就是平滑 SMO 角，不再经过 θ̂�?*/
        theta_park = emf_pll_theta_smooth(&s_emf_pll, hfi_hand_w_el());
    } else if (s_hand_ang > 0.0f) {
        /* 只转收注入前记下�?δ�?*/
        theta_park = hfi_hand_wrap(theta_park + s_hand_ang * s_hand_dth);
    }
#endif
#if M1_HFI_MOTION_BYPASS_ENABLE
    if (hfi_sqwave_get_stage() == HFI_STAGE_RUN) {
        s_hfi_recon_theta = theta_park;
        s_hfi_recon_theta_ok = 1u;
    }
#endif
#ifndef M1_HFI_QKICK_SWEEP_ENABLE
#define M1_HFI_QKICK_SWEEP_ENABLE 0
#endif
#ifndef M1_HFI_QKICK_AFTER_LOCK_ENABLE
#define M1_HFI_QKICK_AFTER_LOCK_ENABLE 0
#endif
#if M1_HFI_QKICK_SWEEP_ENABLE || M1_HFI_QKICK_AFTER_LOCK_ENABLE
    if (hfi_sqwave_consume_pi_reset() != 0u) {
#if (M1_HFI_GATE == 97) || (M1_HFI_GATE == 98) || (M1_HFI_GATE == 101) || \
    (M1_HFI_GATE == 102) || (M1_HFI_GATE == 103) || (M1_HFI_GATE == 104) || \
    (M1_HFI_GATE == 105) || (M1_HFI_GATE == 106) || (M1_HFI_GATE == 107) || \
    (M1_HFI_GATE == 108) || (M1_HFI_GATE == 109) || (M1_HFI_GATE == 110) || \
    (M1_HFI_GATE == 111) || (M1_HFI_GATE == 112) || (M1_HFI_GATE == 113) || \
    (M1_HFI_GATE == 114) || (M1_HFI_GATE == 115) || (M1_HFI_GATE == 116) || \
    (M1_HFI_GATE == 117) || (M1_HFI_GATE == 118) || (M1_HFI_GATE == 119) || \
    (M1_HFI_GATE == 120) || (M1_HFI_GATE == 121) || (M1_HFI_GATE == 122) || \
    (M1_HFI_GATE == 123) || (M1_HFI_GATE == 124) || (M1_HFI_GATE == 125) || \
    (M1_HFI_GATE == 126) || (M1_HFI_GATE == 127) || (M1_HFI_GATE == 128) || (M1_HFI_GATE == 129) || (M1_HFI_GATE == 130) || (M1_HFI_GATE == 131) || (M1_HFI_GATE == 138) || (M1_HFI_GATE == 132) || (M1_HFI_GATE == 133) || (M1_HFI_GATE == 134) || (M1_HFI_GATE == 135) || (M1_HFI_GATE == 136) || (M1_HFI_GATE == 137) || (M1_HFI_GATE == 139) || (M1_HFI_GATE == 140) || (M1_HFI_GATE == 141)
        /* 踢时只清 Iq PI，Id 积分保持 */
        foc_pi_reset(&ctx->pi_iq);
        ctx->uq_pi = 0.0f;
#else
        motor_foc_loop_pi_reset(ctx);
#endif
    }
#endif
#if M1_HFI_MOTION_BYPASS_ENABLE && M1_SPEED_LOOP_ENABLE
    {
#ifndef M1_HFI_OMEGA_FF_FROM_REF
#define M1_HFI_OMEGA_FF_FROM_REF 0
#endif
#ifndef M1_HFI_OMEGA_FF_LOCKED_USE_ENC
#define M1_HFI_OMEGA_FF_LOCKED_USE_ENC 1
#endif
#ifndef M1_HFI_OMEGA_FF_SRC
#define M1_HFI_OMEGA_FF_SRC 1
#endif
#ifndef M1_HFI_QKICK_AFTER_LOCK_ENABLE
#define M1_HFI_QKICK_AFTER_LOCK_ENABLE 0
#endif
#ifndef M1_HFI_QKICK_IF_ENABLE
#define M1_HFI_QKICK_IF_ENABLE 0
#endif
#ifndef M1_HFI_IQ_RAMP_ENABLE
#define M1_HFI_IQ_RAMP_ENABLE 0
#endif
#ifndef M1_HFI_IQ_AUTH_FEED_ENABLE
#define M1_HFI_IQ_AUTH_FEED_ENABLE 0
#endif
#ifndef M1_HFI_IQ_PULL_ENABLE
#define M1_HFI_IQ_PULL_ENABLE 0
#endif
#ifndef M1_HFI_SPD_CLOSE_ENABLE
#define M1_HFI_SPD_CLOSE_ENABLE 0
#endif
#ifndef M1_HFI_IQ_AUTH_FEED_LEGACY_BRANCH
#define M1_HFI_IQ_AUTH_FEED_LEGACY_BRANCH 0
#endif
        float rpm_ff;

        if (hfi_sqwave_speed_run_active() != 0u) {
            const float rpm_cmd = hfi_sqwave_get_speed_ref_rpm();

            ctx->omega_ref = rpm_cmd;
            dbg.outer_omega_ref = rpm_cmd;
            if (ctx->outer_mode != M1_OUTER_SPEED) {
#if M1_HFI_IQ_PULL_ENABLE
                /* 转矩→速度：用拉起电流�?HFI 转速做无扰，避免积分从 0 �?Iq 撤掉 */
                {
                    const float rpm_scale =
                        60.0f / (2.0f * 3.14159265f * (float)M1_POLE_PAIRS);

                    motor_outer_set_mode(ctx, M1_OUTER_SPEED, M1_HFI_IQ_PULL_A,
                                         hfi_sqwave_get_pll_int_el() * rpm_scale);
                }
#elif M1_HFI_SPD_CLOSE_ENABLE
                /* 用正在出力的馈流和当�?ω* 做无扰，Iq 不要�?0 重新�?*/
                motor_outer_set_mode(ctx, M1_OUTER_SPEED, ctx->iq_ref, rpm_cmd);
#else
                /* ω* 已写成目标。bumpless 若看见它，积分会预成 -Kp·目标。冷启动�?0 交接�?*/
                ctx->omega_ref = 0.0f;
                motor_outer_set_mode(ctx, M1_OUTER_SPEED, 0.0f, 0.0f);
                ctx->omega_ref = rpm_cmd;
                dbg.outer_omega_ref = rpm_cmd;
#endif
            }
#if M1_HFI_OMEGA_FF_SRC == 3
            /* S1 自举混合：角前馈通道�?ω_cmd（小权重），主通道�?HFI �?*/
            rpm_ff = rpm_cmd;
#elif M1_HFI_OMEGA_FF_FROM_REF
            /*
             * CAPTURE/FAULT：ω_ff=ω_ref；LOCKED：本档也�?ω_ref（LOCKED_USE_ENC=0）�?
             */
            if ((M1_HFI_OMEGA_FF_LOCKED_USE_ENC != 0) &&
                (hfi_sqwave_get_lock() == HFI_LOCK_LOCKED)) {
                rpm_ff = s_pll_omega_mech_rpm;
            } else {
                rpm_ff = rpm_cmd;
            }
#else
            rpm_ff = s_pll_omega_mech_rpm;
#endif
        } else {
#if M1_HFI_QKICK_AFTER_LOCK_ENABLE || M1_HFI_IQ_RAMP_ENABLE || \
    M1_HFI_IQ_PULL_ENABLE || \
    (M1_HFI_IQ_AUTH_FEED_ENABLE && M1_HFI_IQ_AUTH_FEED_LEGACY_BRANCH)
            /* �?蠕动/I–f/Iq 斜坡/固定Iq拉起，或 C4 旧误接（LEGACY）：速度环关时由 HFI �?ref�?
             * 正确 C4（GATE�?4）不走此支，过线前与 C3b 同为字面 0�?*/
            ctx->id_ref = hfi_sqwave_get_id_ref();
            ctx->iq_ref = hfi_sqwave_get_iq_ref();
#if M1_HFI_IQ_PULL_ENABLE
            /* 拉起�?ω*=0；勿�?get_speed_ref（会空推进阶梯） */
            ctx->omega_ref = 0.0f;
#else
            ctx->omega_ref = hfi_sqwave_get_speed_ref_rpm();
#endif
            dbg.outer_omega_ref = ctx->omega_ref;
#else
            ctx->id_ref = 0.0f;
            ctx->iq_ref = 0.0f;
            ctx->omega_ref = 0.0f;
            dbg.outer_omega_ref = 0.0f;
#endif
            if (ctx->outer_mode != M1_OUTER_DISABLED) {
                motor_outer_set_mode(ctx, M1_OUTER_DISABLED, 0.0f, 0.0f);
            }
#if M1_HFI_QKICK_IF_ENABLE
            /* I–f：θ�?前馈跟开�?ω_cmd，避免只�?enc 时起步滞�?*/
            if (hfi_sqwave_if_leave_active() != 0u) {
                rpm_ff = ctx->omega_ref;
            } else
#endif
#if M1_HFI_OMEGA_FF_FROM_REF
            {
                rpm_ff = 0.0f;
            }
#else
            {
                rpm_ff = s_pll_omega_mech_rpm;
            }
#endif
        }
#if M1_HFI_OMEGA_FF_SRC == 0
        /* S2（M1_HFI_GATE=2）：禁止 enc PLL 转速进 HFI。S1 同样 SRC=0�?*/
        (void)rpm_ff;
        hfi_sqwave_set_omega_ff_el(0.0f);
#else
        {
            const float omega_ff_el = rpm_ff * (0.104719755f) * (float)M1_POLE_PAIRS;
            hfi_sqwave_set_omega_ff_el(omega_ff_el);
        }
#endif
    }
#elif M1_HFI_PARK_ENABLE
    ctx->id_ref = 0.0f;
    ctx->iq_ref = hfi_sqwave_get_iq_ref();
#endif
#endif
    dbg.foc_theta_el = theta_park;
#if M1_ENC_OPTIONAL_ENABLE || (M1_IF_ENABLE && M1_IF_TO_OBS_ENABLE)
    s_theta_park_last = theta_park;
#endif

    motor_trig_sincos(theta_park, &cos_el, &sin_el);
    Park_Transform_sc(i_alpha, i_beta, sin_el, cos_el, &id, &iq);
    dbg.foc_id = id;
    dbg.foc_id_lpf = id; /* CURRENT_LOOP + ID_PI_LPF 时下面会覆盖 */
    dbg.foc_iq = iq;

#if M1_HFI_ENABLE
    hfi_sqwave_on_current(id, iq, i_alpha, i_beta);
#if M1_HFI_IQ_AUTH_ENABLE
    /* 解调后更新权威：限速环 Iq，并�?PI 上下限防积分顶满 */
    if (hfi_sqwave_speed_run_active() != 0u) {
        const float lim = hfi_sqwave_get_iq_auth_abs();

        if (ctx->iq_ref > lim) {
            ctx->iq_ref = lim;
        } else if (ctx->iq_ref < -lim) {
            ctx->iq_ref = -lim;
        }
        ctx->pi_speed.out_max = lim;
        ctx->pi_speed.out_min = -lim;
        ctx->pi_speed.int_max = lim;
        ctx->pi_speed.int_min = -lim;
        if (ctx->pi_speed.integrator > lim) {
            ctx->pi_speed.integrator = lim;
        } else if (ctx->pi_speed.integrator < -lim) {
            ctx->pi_speed.integrator = -lim;
        }
    } else {
        ctx->pi_speed.out_max = M1_SPEED_PI_OUT_MAX;
        ctx->pi_speed.out_min = M1_SPEED_PI_OUT_MIN;
        ctx->pi_speed.int_max = M1_SPEED_PI_INT_MAX;
        ctx->pi_speed.int_min = M1_SPEED_PI_INT_MIN;
    }
#endif
#ifndef M1_HFI_IQ_AUTH_FEED_ENABLE
#define M1_HFI_IQ_AUTH_FEED_ENABLE 0
#endif
#ifndef M1_HFI_IQ_AUTH_FEED_LEGACY_BRANCH
#define M1_HFI_IQ_AUTH_FEED_LEGACY_BRANCH 0
#endif
#if M1_HFI_IQ_AUTH_FEED_ENABLE && !M1_HFI_IQ_AUTH_FEED_LEGACY_BRANCH
    /* C4 正确接法：解�?权威之后再写 Iq；未 ok �?get_iq_ref=0，与 C3b �?*/
    if (hfi_sqwave_speed_run_active() == 0u) {
        ctx->iq_ref = hfi_sqwave_get_iq_ref();
    }
#endif
#if M1_HFI_SMO_HAND_ENABLE && M1_EMF_PLL_ENABLE && M1_HFI_HAND_IQ_HOLD_ON_IDUP
    hfi_hand_iq_hold_apply(ctx);
#endif
#if M1_HFI_SMO_HAND_ENABLE && M1_EMF_PLL_ENABLE && M1_HFI_HAND_DECEL_BRAKE_ENABLE
    hfi_hand_decel_brake_apply(ctx);
#endif
#if (M1_HFI_GATE == 79) && M1_HFI_SMO_HAND_ENABLE && M1_EMF_PLL_ENABLE
    /* S2：窗内钉 Id*=0；Id PI 仍旁路（硬关对照） */
    if ((s_vesc_win_smo != 0u) && (s_hand_ok != 0u) &&
        (s_hand_state == HFI_HAND_SMO)) {
        ctx->id_ref = 0.0f;
        dbg.foc_id_ref = 0.0f;
    }
#endif
#endif

#if M1_HFI_ENABLE
#ifndef M1_HFI_HAND_ID_WITH_VH
#ifndef M1_HFI_HAND_ID_OVERLAP_ENABLE
#define M1_HFI_HAND_ID_OVERLAP_ENABLE 0
#endif
#ifndef M1_HFI_HAND_OPEN_ID_ENABLE
#define M1_HFI_HAND_OPEN_ID_ENABLE 0
#endif
#ifndef M1_HFI_VESC_ID_HANDOFF_ENABLE
#define M1_HFI_VESC_ID_HANDOFF_ENABLE 0
#endif
#ifndef M1_HFI_HFI_ID_SOFT_ENABLE
#define M1_HFI_HFI_ID_SOFT_ENABLE 0
#endif
#ifndef M1_HFI_ID_ON_FROM_RUN_ENABLE
#define M1_HFI_ID_ON_FROM_RUN_ENABLE 0
#endif
#if M1_HFI_HAND_ID_OVERLAP_ENABLE || M1_HFI_HAND_OPEN_ID_ENABLE || \
    M1_HFI_VESC_ID_HANDOFF_ENABLE || M1_HFI_HFI_ID_SOFT_ENABLE || \
    M1_HFI_ID_ON_FROM_RUN_ENABLE
#define M1_HFI_HAND_ID_WITH_VH 1
#else
#define M1_HFI_HAND_ID_WITH_VH 0
#endif
#endif
#if M1_HFI_HAND_ID_WITH_VH
    /*
     * 无扰：id* = (1-soft)·id → 误差 = -soft·id；
     * soft=0 时误差为 0；soft→1 → id*→0。须在 foc_loop 之前。
     */
    if (hfi_sqwave_id_pi_bypass() == 0u) {
        const float soft = hfi_sqwave_id_pi_soft_scale();

        if (soft < 1.0f) {
            ctx->id_ref = (1.0f - soft) * id;
        } else {
            ctx->id_ref = 0.0f;
        }
    }
#endif
#endif

    motor_foc_loop_dbg_id_ref(ctx);
    motor_current_update_acdc(id, iq);

    if (ctx->mode == M1_CTRL_CURRENT_LOOP) {
        float id_for_pi = id;
#if M1_HFI_ENABLE
#ifndef M1_HFI_ID_PI_LPF_ENABLE
#define M1_HFI_ID_PI_LPF_ENABLE 0
#endif
#ifndef M1_HFI_ID_PI_LPF_A
#define M1_HFI_ID_PI_LPF_A (0.05f)
#endif
#if M1_HFI_ID_PI_LPF_ENABLE
        /*
         * Id PI 只追慢 Id；HFI 解调仍用上面的裸 id。
         * 使 Ud_pi 少打方波频段，带载时更接近旧 ID_PI_OFF+FEED。
         */
        {
            static float s_id_pi_lpf;

            if (hfi_sqwave_id_pi_bypass() == 0u) {
                s_id_pi_lpf += M1_HFI_ID_PI_LPF_A * (id - s_id_pi_lpf);
                id_for_pi = s_id_pi_lpf;
            } else {
                s_id_pi_lpf = id;
            }
            dbg.foc_id_lpf = s_id_pi_lpf;
        }
#else
        dbg.foc_id_lpf = id;
#endif
#else
        dbg.foc_id_lpf = id;
#endif
#if !M1_IF_ENABLE
        motor_startup_finish_tick(ctx, iq, &startup);
#endif
#if M1_HFI_ROTATE_PI_ENABLE && M1_HFI_SMO_HAND_ENABLE
        if (s_hand_ang < 1.0f) {
            s_rot_armed = 1u;
        }
        if ((s_rot_armed != 0u) && (s_hand_ang >= 1.0f) &&
            (s_rot_th_ok != 0u)) {
            hfi_hand_rotate_current_pi(ctx,
                                       hfi_hand_wrap(theta_park - s_rot_th_prev),
                                       id_for_pi, iq);
            s_rot_armed = 0u;
        }
        s_rot_th_prev = theta_park;
        s_rot_th_ok = 1u;
#endif
        motor_foc_loop_tick(ctx, id_for_pi, iq, &startup, theta_enc_park);
    }

    if (ctx->mode == M1_CTRL_OBSERVE_ONLY || ctx->mode == M1_CTRL_OPEN_LOOP) {
        motor_open_sweep_tick(ctx);
    }

    switch (ctx->mode) {
    case M1_CTRL_CURRENT_LOOP:
#if M1_LD_LQ_IDENT_ENABLE
        if (deadband_id_cal_in_ld_lq_ident()) {
#if M1_LD_LQ_IDENT_OPEN_LOOP_ENABLE
            if (ld_lq_ident_inject_active()) {
                ud_out = ld_lq_ident_u_bias_d() + ld_lq_ident_u_inj_d();
                uq_out = ld_lq_ident_u_bias_q() + ld_lq_ident_u_inj_q();
            } else {
                ud_out = ctx->ud_pi;
                uq_out = ctx->uq_pi;
            }
#else
            ud_out = ctx->ud_pi + ld_lq_ident_u_inj_d();
            uq_out = ctx->uq_pi + ld_lq_ident_u_inj_q();
#endif
            ld_lq_ident_integrate(id, iq, ud_out, uq_out);
            break;
        }
#endif
#if M1_ID_CAL_ALIGN_ENABLE && M1_ID_CAL_FIX_THETA_ENABLE
        if (deadband_id_cal_use_align_ud()) {
#if M1_ID_CAL_DUAL_ANGLE_ENABLE
            ud_out = deadband_id_cal_align_ud_v();
#else
            ud_out = M1_ID_CAL_ALIGN_UD_V;
#endif
            uq_out = 0.0f;
        } else if (deadband_id_cal_use_post_ident_hold_ud()) {
            ud_out = M1_ID_CAL_ALIGN_UD_V;
            uq_out = 0.0f;
        } else
#endif
#if M1_LD_LQ_IDENT_ENABLE
        if (deadband_id_cal_use_ld_lq_align_ud()) {
            ud_out = M1_ID_CAL_ALIGN_UD_V;
            uq_out = 0.0f;
        } else
#endif
        if (startup.use_fixed_uq) {
            ud_out = 0.0f;
            uq_out = startup.uq_out;
        } else {
            ud_out = ctx->ud_pi + deadband_service_ud_inject(id);
            uq_out = ctx->uq_pi;
#if M1_HFI_ENABLE
            /* 旁路：清 Id。放行后：Ud*=soft；OVERLAP 不清积分（id* 无扰）�?*/
            if (hfi_sqwave_id_pi_bypass() != 0u) {
                foc_pi_reset(&ctx->pi_id);
                ctx->ud_pi = 0.0f;
                dbg.foc_ud_pi = 0.0f;
                ud_out = deadband_service_ud_inject(id);
            } else {
                float soft = hfi_sqwave_id_pi_soft_scale();
                float ud_db = deadband_service_ud_inject(id);

#if !M1_HFI_HAND_ID_WITH_VH
                if (soft < 1.0f) {
                    foc_pi_reset(&ctx->pi_id);
                }
#endif
                ud_out = ud_db + soft * ctx->ud_pi;
                dbg.foc_ud_pi = soft * ctx->ud_pi;
            }
#endif
        }
#if M1_HFI_ENABLE
        {
            float ud_ov;
            float uq_ov;
            float ud_inj;
            float uq_inj;

            if (hfi_sqwave_override_voltage(&ud_ov, &uq_ov) != 0u) {
                ud_out = ud_ov;
                uq_out = uq_ov;
                /*
                 * IDLE/DONE 等强制电压时 PI 输出被丢弃；若不卸积分，
                 * 再进 RUN 会带着饱和 Ud_pi（C4k 1300：IDLE~13.8V→PRE 解调崩）�?
                 */
                if ((ud_ov == 0.0f) && (uq_ov == 0.0f)) {
                    foc_pi_reset(&ctx->pi_id);
                    foc_pi_reset(&ctx->pi_iq);
                    ctx->ud_pi = 0.0f;
                    ctx->uq_pi = 0.0f;
                    dbg.foc_ud_pi = 0.0f;
                    dbg.foc_uq_pi = 0.0f;
                }
            } else {
                hfi_sqwave_get_inj(&ud_inj, &uq_inj);
                ud_out += ud_inj;
                uq_out += uq_inj;
                /* 进桥的载波，供 SMO 减 Vh；与本拍 Park 同框 */
                ua_hfi = ud_inj * cos_el - uq_inj * sin_el;
                ub_hfi = ud_inj * sin_el + uq_inj * cos_el;
#if M1_HFI_INJECT_AB_ENABLE || M1_HFI_INJECT_POST_LOOP
                {
                    float ua_inj;
                    float ub_inj;

                    /* ±Vh 沿 θ̂ 在定子上；旋回本拍 Park 再进 SVPWM */
                    hfi_sqwave_get_inj_ab(&ua_inj, &ub_inj);
                    ud_out += ua_inj * cos_el + ub_inj * sin_el;
                    uq_out += -ua_inj * sin_el + ub_inj * cos_el;
                    ua_hfi += ua_inj;
                    ub_hfi += ub_inj;
                }
#endif
            }
        }
#endif
        break;
    case M1_CTRL_OBSERVE_ONLY:
        ud_out = 0.0f;
        uq_out = ctx->uq_open;
        break;
    case M1_CTRL_OPEN_LOOP:
#if (M1_OPEN_UQ_PRE_ID_CAL_ENABLE || M1_OPEN_UD_PRE_ID_CAL_ENABLE) && \
    M1_OPEN_PRE_ID_LADDER_ALIGN_ENABLE
        if (motor_open_sweep_in_pre_id_align()) {
            ud_out = M1_OPEN_PRE_ID_LADDER_ALIGN_UD_V;
            uq_out = 0.0f;
            break;
        }
#endif
#if M1_OPEN_UD_PRE_ID_CAL_ENABLE
        if (motor_open_sweep_in_pre_id_ud_ladder()) {
            ud_out = ctx->ud_open;
            uq_out = 0.0f;
            break;
        }
#endif
        ud_out = 0.0f;
        uq_out = ctx->uq_open;
        break;
    default:
        ud_out = 0.0f;
        uq_out = 0.0f;
        break;
    }

    dbg.foc_uq_out = uq_out;
    dbg.foc_ud_out = ud_out;

#if M1_HFI_ENABLE
#ifndef M1_HFI_DELTA_SWEEP_ENABLE
#define M1_HFI_DELTA_SWEEP_ENABLE 0
#endif
#ifndef M1_HFI_IPD_SWEEP_ENABLE
#define M1_HFI_IPD_SWEEP_ENABLE 0
#endif
#ifndef M1_HFI_QKICK_SWEEP_ENABLE
#define M1_HFI_QKICK_SWEEP_ENABLE 0
#endif
#ifndef M1_HFI_QKICK_AFTER_LOCK_ENABLE
#define M1_HFI_QKICK_AFTER_LOCK_ENABLE 0
#endif
#ifndef M1_HFI_AXIS_SEL_ENABLE
#define M1_HFI_AXIS_SEL_ENABLE 0
#endif
#if M1_HFI_DELTA_SWEEP_ENABLE || M1_HFI_IPD_SWEEP_ENABLE || \
    M1_HFI_QKICK_SWEEP_ENABLE || M1_HFI_QKICK_AFTER_LOCK_ENABLE
    dbg.hfi_theta_cmd = hfi_sqwave_get_theta_cmd(); /* ch1: θ_cmd/δ [rad] */
#else
    dbg.hfi_theta_cmd = ctx->omega_ref; /* ch1: ω_ref [rpm] */
#endif
    dbg.hfi_theta_hat = hfi_sqwave_get_theta_hat();
    dbg.hfi_theta_err = hfi_sqwave_get_theta_err();
    dbg.hfi_eps = hfi_sqwave_get_eps();
    dbg.hfi_di_q = hfi_sqwave_get_di_q();
    dbg.hfi_di_d = hfi_sqwave_get_di_d();
    dbg.hfi_x_raw = hfi_sqwave_get_x_raw();
    dbg.hfi_y_raw = hfi_sqwave_get_y_raw();
    dbg.hfi_vh_sign = hfi_sqwave_get_vh_sign();
#if M1_HFI_DELTA_SWEEP_ENABLE
    dbg.hfi_stage = (float)hfi_sqwave_get_stage();
#else
    dbg.hfi_stage = (float)hfi_sqwave_get_stage() + hfi_sqwave_get_eps_dead();
#endif
    dbg.hfi_lock = (float)hfi_sqwave_get_lock();
    {
        const float rpm_scale = 60.0f / (2.0f * 3.14159265f * (float)M1_POLE_PAIRS);

#ifndef M1_HFI_SPEED_OBS_INT
#define M1_HFI_SPEED_OBS_INT 0
#endif
#if M1_HFI_SPEED_OBS_INT
        /* 速度观测 = 积分项。角度仍�?Kp·eps+�?推进�?*/
        dbg.hfi_omega_rpm = hfi_sqwave_get_pll_int_el() * rpm_scale;
#else
        dbg.hfi_omega_rpm = hfi_sqwave_get_omega_el() * rpm_scale;
#endif
#if ((M1_HFI_GATE == 38) || (M1_HFI_GATE == 53) || (M1_HFI_GATE == 54) || \
     (M1_HFI_GATE == 55) || (M1_HFI_GATE == 56) || (M1_HFI_GATE == 57) || (M1_HFI_GATE == 58) || (M1_HFI_GATE == 59) || (M1_HFI_GATE == 60) || (M1_HFI_GATE == 61) || (M1_HFI_GATE == 62) || (M1_HFI_GATE == 63) || (M1_HFI_GATE == 64) || (M1_HFI_GATE == 65) || (M1_HFI_GATE == 66) || (M1_HFI_GATE == 67) || (M1_HFI_GATE == 68) || (M1_HFI_GATE == 69) || (M1_HFI_GATE == 70) || (M1_HFI_GATE == 71) || (M1_HFI_GATE == 72) || (M1_HFI_GATE == 73) || (M1_HFI_GATE == 74) || (M1_HFI_GATE == 75) || (M1_HFI_GATE == 76) || (M1_HFI_GATE == 77) || (M1_HFI_GATE == 78) || (M1_HFI_GATE == 79) || (M1_HFI_GATE == 80) || (M1_HFI_GATE == 91) || (M1_HFI_GATE == 92) || (M1_HFI_GATE == 93)) && \
    M1_SPEED_LOOP_ENABLE
        hfi_spd_shadow_step(ctx->iq_ref, dbg.hfi_omega_rpm);
#endif
#if M1_HFI_IPD_SWEEP_ENABLE
        /* IPD 扫压：trim 通道塞脉�?Ud [V]，勿�?rpm 比例 */
        dbg.hfi_omega_trim_rpm = hfi_sqwave_get_omega_trim_el();
        dbg.hfi_ipd_phase = (float)hfi_sqwave_get_ipd_phase();
        dbg.hfi_ipd_pulse_ud = hfi_sqwave_get_ipd_pulse_ud();
        dbg.hfi_qkick_seed = 0.0f;
        dbg.hfi_qkick_dth_deg = 0.0f;
        dbg.hfi_qkick_verdict = 0.0f;
#elif M1_HFI_DELTA_SWEEP_ENABLE
        /* δ 标定：ch9 = di_d 解调 */
        dbg.hfi_omega_trim_rpm = hfi_sqwave_get_eps_d();
        dbg.hfi_ipd_phase = 0.0f;
        dbg.hfi_ipd_pulse_ud = 0.0f;
        dbg.hfi_qkick_seed = 0.0f;
        dbg.hfi_qkick_dth_deg = 0.0f;
        dbg.hfi_qkick_verdict = 0.0f;
#elif M1_HFI_QKICK_SWEEP_ENABLE || M1_HFI_QKICK_AFTER_LOCK_ENABLE
#ifndef M1_HFI_SENSED_CAL_ENABLE
#define M1_HFI_SENSED_CAL_ENABLE 0
#endif
#if M1_HFI_SENSED_CAL_ENABLE
        /* 有感标定：trim = PLL 修正 dw [el rad/s] �?机械 rpm */
        dbg.hfi_omega_trim_rpm = hfi_sqwave_get_omega_trim_el() * rpm_scale;
        dbg.hfi_qkick_verdict = hfi_sqwave_get_qkick_verdict();
#else
        /* 台架：ch9 = eps_d + 0.1·flip + 0.01·axis_ok */
        dbg.hfi_omega_trim_rpm =
            hfi_sqwave_get_eps_d() + 0.1f * hfi_sqwave_get_axis_flip_n() +
            ((hfi_sqwave_axis_ok() != 0u) ? 0.01f : 0.0f);
        dbg.hfi_qkick_verdict = hfi_sqwave_get_pll_int_el() * rpm_scale;
#endif
        dbg.hfi_ipd_phase = (float)hfi_sqwave_get_qkick_phase();
        dbg.hfi_ipd_pulse_ud = hfi_sqwave_get_ipd_pulse_ud();
        dbg.hfi_qkick_seed = hfi_sqwave_get_qkick_seed();
        dbg.hfi_qkick_dth_deg = hfi_sqwave_get_qkick_dth() * (180.0f / 3.14159265f);
#elif M1_HFI_AXIS_SEL_ENABLE
        /* 静置定轴：ch9 同号 */
        dbg.hfi_omega_trim_rpm =
            hfi_sqwave_get_eps_d() + 0.1f * hfi_sqwave_get_axis_flip_n() +
            ((hfi_sqwave_axis_ok() != 0u) ? 0.01f : 0.0f);
        dbg.hfi_ipd_phase = 0.0f;
        dbg.hfi_ipd_pulse_ud = 0.0f;
        dbg.hfi_qkick_seed = 0.0f;
        dbg.hfi_qkick_dth_deg = 0.0f;
        dbg.hfi_qkick_verdict = hfi_sqwave_get_pll_int_el() * rpm_scale;
#else
        dbg.hfi_omega_trim_rpm = hfi_sqwave_get_omega_trim_el() * rpm_scale;
        dbg.hfi_ipd_phase = 0.0f;
        dbg.hfi_ipd_pulse_ud = 0.0f;
        dbg.hfi_qkick_seed = 0.0f;
        dbg.hfi_qkick_dth_deg = 0.0f;
        dbg.hfi_qkick_verdict = 0.0f;
#endif
    }
    dbg.open_seq_phase = (uint8_t)hfi_sqwave_get_stage();
#endif

    t_pre_obs = *(volatile uint32_t *)&DWT->CYCCNT;
#if M1_EMF_VEQ_ENABLE || M1_EMF_SMO_ENABLE || M1_EMF_PLL_ENABLE
    {
        float omega_mech_rpm = 0.0f;
        float u_alpha;
        float u_beta;
        float theta_obs_ref = theta_enc_park;
        uint8_t smo_hold = 0u;

#if M1_PLL_ENABLE
        omega_mech_rpm = s_speed_fb_rpm;
#endif
#if M1_HFI_ENABLE && M1_HFI_MOTION_BYPASS_ENABLE && M1_EMF_SMO_ENABLE && \
    M1_EMF_PLL_ENABLE
        /* 旁路相对 θ̂_hfi。速度环未接管前每个周期清掉，避免零速假转速和编码器初值�?*/
        theta_obs_ref = hfi_sqwave_get_theta_hat();
#if M1_HFI_SMO_SUB_VH_ENABLE
        /* 并行观测：只在极性翻面清状态，HFI 段继续跑 SMO */
        if (hfi_sqwave_take_polarity_flip() != 0u) {
#else
        if ((hfi_sqwave_speed_run_active() == 0u) ||
            (hfi_sqwave_take_polarity_flip() != 0u)) {
#endif
            smo_hold = 1u;
            emf_smo_reset(&s_emf_smo);
            emf_pll_reset(&s_emf_pll);
            hfi_smo_w_ma_reset();
#if M1_HFI_SMO_HAND_ENABLE && M1_EMF_PLL_ENABLE
            s_hand_ok = 0u;
#endif
            dbg.obs_pll_theta_hat = 0.0f;
            dbg.obs_pll_theta_err = 0.0f;
            dbg.obs_pll_omega_el = 0.0f;
            dbg.obs_spd_pll_rpm = 0.0f;
            dbg.obs_spd_rpm_err = 0.0f;
        }
#endif
        if (smo_hold != 0u) {
            /* 本拍不更�?*/
        } else {
        /* 与电流环同思路：反 Park 复用本拍 Park �?sin/cos（θ_park 帧下�?ud/uq�?*/
        u_alpha = ud_out * cos_el - uq_out * sin_el;
        u_beta = ud_out * sin_el + uq_out * cos_el;
#if M1_HFI_SMO_SUB_VH_ENABLE
        /* 桥上仍叠 Vh；磁链/SMO 只吃基波 */
        u_alpha -= ua_hfi;
        u_beta -= ub_hfi;
#endif

#if M1_EMF_VEQ_ENABLE
        emf_veq_update(&s_emf_veq, i_alpha, i_beta, u_alpha, u_beta,
                       theta_enc_park, omega_mech_rpm);
        dbg.obs_i_alpha = s_emf_veq.i_alpha;
        dbg.obs_i_beta = s_emf_veq.i_beta;
        dbg.obs_u_alpha = s_emf_veq.u_alpha;
        dbg.obs_u_beta = s_emf_veq.u_beta;
        dbg.obs_e_alpha = s_emf_veq.e_alpha;
        dbg.obs_e_beta = s_emf_veq.e_beta;
        dbg.obs_theta_hat = s_emf_veq.theta_hat;
        dbg.obs_theta_err = s_emf_veq.theta_err;
        dbg.obs_emag = s_emf_veq.emag;
        dbg.obs_omega_el = s_emf_veq.omega_el;
        dbg.obs_psi_inst = s_emf_veq.psi_inst;
#endif
#if M1_EMF_SMO_ENABLE
        {
            const uint8_t lpf_band = emf_smo_lpf_sched_update(omega_mech_rpm);

            emf_smo_update(&s_emf_smo, i_alpha, i_beta, u_alpha, u_beta,
                           theta_obs_ref, omega_mech_rpm);
            dbg.obs_smo_e_alpha = s_emf_smo.e_alpha;
            dbg.obs_smo_e_beta = s_emf_smo.e_beta;
            dbg.obs_smo_theta_hat = s_emf_smo.theta_hat;
            dbg.obs_smo_theta_err = s_emf_smo.theta_err;
            dbg.obs_smo_emag = s_emf_smo.emag;
            dbg.obs_smo_lpf_hz = emf_smo_get_lpf_hz();
            dbg.obs_smo_lpf_band = (float)lpf_band;
        }
#if !M1_EMF_VEQ_ENABLE
        dbg.obs_i_alpha = i_alpha;
        dbg.obs_i_beta = i_beta;
        dbg.obs_u_alpha = u_alpha;
        dbg.obs_u_beta = u_beta;
        dbg.obs_omega_el = s_emf_smo.omega_el;
#endif
#endif
#if M1_EMF_PLL_ENABLE
#if M1_EMF_PLL_USE_SMO
        emf_pll_update(&s_emf_pll,
                       s_emf_smo.e_alpha, s_emf_smo.e_beta,
                       theta_obs_ref, M1_CTRL_TS_S);
#else
        emf_pll_update(&s_emf_pll,
                       s_emf_veq.e_alpha, s_emf_veq.e_beta,
                       theta_obs_ref, M1_CTRL_TS_S);
#endif
        dbg.obs_pll_theta_hat = s_emf_pll.theta_hat;
        dbg.obs_pll_theta_err = s_emf_pll.theta_err;
        dbg.obs_pll_omega_el = s_emf_pll.omega_el;
        dbg.obs_pll_pd = s_emf_pll.last_pd;
#if M1_HFI_ENABLE && M1_HFI_MOTION_BYPASS_ENABLE && M1_EMF_PLL_ENABLE
        {
            float dth = s_emf_pll.theta_hat - theta_obs_ref;
            const float rpm_scale =
                60.0f / (6.28318530718f * (float)M1_POLE_PAIRS);

            while (dth > 3.14159265f) {
                dth -= 6.2831853f;
            }
            while (dth < -3.14159265f) {
                dth += 6.2831853f;
            }
            /* 旁路角差 = θ_smo �?θ_hfi。速度环不读这个量�?*/
            dbg.obs_pll_theta_err = dth;
            dbg.obs_spd_rpm_err = s_emf_pll.omega_el * rpm_scale;
            dbg.obs_spd_pll_rpm = hfi_smo_w_ma_step(dbg.obs_spd_rpm_err);
#if (M1_HFI_GATE == 131) || (M1_HFI_GATE == 138) || (M1_HFI_GATE == 141)
            hfi_pub_step(s_emf_pll.theta_hat, s_emf_pll.omega_el, dth);
#endif
#if M1_HFI_SMO_HAND_ENABLE && M1_EMF_PLL_ENABLE
            s_hand_th = s_emf_pll.theta_hat;
            s_hand_w = dbg.obs_spd_pll_rpm;
            s_hand_ok = 1u;
#endif
        }
#endif
#if M1_OBS_SPD_PLL_ENABLE && M1_PLL_ENABLE
        /* θ̂ �?速度环同�?PLL @ 2 kHz（省 ISR）；有感段只预热，OBS 时进速度�?*/
        if (s_emf_pll.primed != 0u) {
            const float pp = (float)M1_POLE_PAIRS;

            if (s_obs_spd_pll_primed == 0u) {
                motor_pll_reset(&s_obs_spd_pll, s_emf_pll.theta_hat);
                s_obs_spd_pll_primed = 1u;
                s_obs_spd_div = 0u;
            }
            if (++s_obs_spd_div >= M1_SPEED_DECIM) {
                float rpm_obs;

                s_obs_spd_div = 0u;
                motor_pll_update(&s_obs_spd_pll, s_emf_pll.theta_hat,
                                 M1_SPEED_TS_S);
                rpm_obs = motor_pll_get_omega_mech(&s_obs_spd_pll) * 60.0f /
                          (6.28318530718f * pp);
                s_obs_spd_pll_rpm = rpm_obs;
                dbg.obs_spd_pll_rpm = rpm_obs;
                dbg.obs_spd_pll_err_rad =
                    motor_pll_get_last_err(&s_obs_spd_pll);
            }
        }
#if M1_OBS_SOFT_SWITCH_ENABLE && M1_OBS_SS_SPEED_SWITCH_ENABLE
        /* 与速度环反馈一致（先角后速时 OBS 初期仍为编码器） */
        dbg.outer_omega_mech_rpm = s_speed_fb_rpm;
#endif
        dbg.obs_spd_rpm_err = s_obs_spd_pll_rpm - s_pll_omega_mech_rpm;
#endif
#endif
        }
    }
#endif
    t_post_obs = *(volatile uint32_t *)&DWT->CYCCNT;
    g_telem_dbg.obs_delta = t_post_obs - t_pre_obs;

    foc_svpwm_apply_abc(axis, uq_out, ud_out, theta_park, ia, ib, ic, id, iq);
    encoder_kick(axis->enc);
    t_post_svpwm = *(volatile uint32_t *)&DWT->CYCCNT;
    /* FOC 核心 = Clarke→PI + SVPWM/kick，不含观测器 */
    g_telem_dbg.foc_delta = (t_pre_obs - foc_t0) + (t_post_svpwm - t_post_obs);

    telem_bringup_tick();

    {
        volatile uint32_t isr_t1 = *(volatile uint32_t *)&DWT->CYCCNT;

        g_telem_dbg.isr_t0 = isr_t0;
        g_telem_dbg.isr_t1 = isr_t1;
        g_telem_dbg.cyccnt_end = isr_t1;
        g_telem_dbg.isr_delta = isr_t1 - isr_t0;
    }
}
