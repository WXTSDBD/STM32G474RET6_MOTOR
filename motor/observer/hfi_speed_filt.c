/**
 * @file hfi_speed_filt.c
 * @brief P5: HFI helpers moved from motor_current.c (call order unchanged).
 */
#include "observer/obs_cfg.h"
#include "motor_math.h"
#include "motor_context.h"
#include "motor_trig.h"
#include "foc_pi.h"
#include "observer/hfi_sqwave.h"
#include "observer/emf_pll.h"
#include "observer/hfi_current_priv.h"

#if (M1_HFI_GATE == 132) || (M1_HFI_GATE == 133) || (M1_HFI_GATE == 134) || (M1_HFI_GATE == 135) || (M1_HFI_GATE == 136) || (M1_HFI_GATE == 137)
/* 速度环反馈上的两个陷波器。不进角度 PLL。指令 40..250 rpm 才开。 */
static uint8_t s_spd_notch_on;
static float s_notch_cmd = -1.0f;
float s_n12_b0, s_n12_b1, s_n12_b2, s_n12_a1, s_n12_a2;
float s_n12_x1, s_n12_x2, s_n12_y1, s_n12_y2;
float s_n14_b0, s_n14_b1, s_n14_b2, s_n14_a1, s_n14_a2;
float s_n14_x1, s_n14_x2, s_n14_y1, s_n14_y2;

void hfi_notch_coeff(float f_hz, float q, float fs,
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

void hfi_notch_prime(float x, float *x1, float *x2, float *y1, float *y2)
{
    *x1 = x;
    *x2 = x;
    *y1 = x;
    *y2 = x;
}

float hfi_notch_run(float x,
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

float hfi_spd_notch_rpm(float rpm, float cmd)
{
    const float fs = 1.0f / OBS_CTRL_TS_S;
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

uint8_t hfi_spd_notch_on(void)
{
    return s_spd_notch_on;
}
#endif

#if M1_HFI_GATE == 140
/* 100 rpm 指令才开。按实测转速陷掉每转 24 次和 28 次，12 次和 14 次留着。 */
static uint8_t s_n24_on;
static float s_n24_rpm = -1.0f;
static float s_n24_abs;
float s_n24_b0, s_n24_b1, s_n24_b2, s_n24_a1, s_n24_a2;
float s_n24_x1, s_n24_x2, s_n24_y1, s_n24_y2;
float s_n28_b0, s_n28_b1, s_n28_b2, s_n28_a1, s_n28_a2;
float s_n28_x1, s_n28_x2, s_n28_y1, s_n28_y2;

void hfi_n24_coeff(float f_hz, float q, float fs,
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

void hfi_n24_prime(float x, float *x1, float *x2, float *y1, float *y2)
{
    *x1 = x;
    *x2 = x;
    *y1 = x;
    *y2 = x;
}

float hfi_n24_run(float x,
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

float hfi_spd_notch24_rpm(float rpm, float cmd)
{
    const float fs = 1.0f / OBS_CTRL_TS_S;
    float cmd_abs = cmd;
    float w;
    float df;
    const float a = OBS_CTRL_TS_S / (0.15f + OBS_CTRL_TS_S);

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
float s_ld_b0, s_ld_b1, s_ld_a1;
float s_ld_x1, s_ld_y1;
static float s_ld_f = -1.0f;
static uint8_t s_ld_on;

void hfi_lead_prime(float x)
{
    s_ld_x1 = x;
    s_ld_y1 = x;
}

void hfi_lead_coeff(float f_hz)
{
    const float fs = 1.0f / OBS_CTRL_TS_S;
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
    root = motor_sqrt4(alpha);
    wz = w0 / root;
    wp = w0 * root;
    c = 2.0f * fs;
    a0 = 1.0f + (c / wp);
    s_ld_b0 = (1.0f + (c / wz)) / a0;
    s_ld_b1 = (1.0f - (c / wz)) / a0;
    s_ld_a1 = (1.0f - (c / wp)) / a0;
    s_ld_f = f_hz;
}

float hfi_spd_lead_rpm(float rpm, float cmd)
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
    s_lead_rpm += (OBS_CTRL_TS_S / 0.15f) * (wabs - s_lead_rpm);
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

float hfi_spd_lead_rep(void)
{
    return s_lead_rep;
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

float hfi_ph_atan(float x)
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

void hfi_ph_pair(float rpm, float p, float phi,
                        float *c, float *s, float *h, float *hadv)
{
    const float a = OBS_CTRL_TS_S / 0.25f;
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

float hfi_spd_phase_rpm(float rpm, float cmd)
{
    const float th = hfi_sqwave_get_theta_hat();
    const float pp = (float)OBS_POLE_PAIRS;
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
    s_ph_rpm += (OBS_CTRL_TS_S / 0.15f) * (wabs - s_ph_rpm);
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
        s_ph_dc += (OBS_CTRL_TS_S / 0.15f) * (rpm - s_ph_dc);
        xdem = rpm - s_ph_dc;
#endif
        hfi_ph_pair(xdem, motor_wrap_pi(th * (12.0f / pp)),
                    hfi_ph_atan(6.28318530718f * (12.0f * s_ph_rpm / 60.0f) * 0.005f),
                    &s_ph12_c, &s_ph12_s, &h12, &h12a);
        hfi_ph_pair(xdem, motor_wrap_pi(th * (14.0f / pp)),
                    hfi_ph_atan(6.28318530718f * (14.0f * s_ph_rpm / 60.0f) * 0.005f),
                    &s_ph14_c, &s_ph14_s, &h14, &h14a);
#if M1_HFI_GATE == 137
        hfi_ph_pair(xdem, motor_wrap_pi(th * (24.0f / pp)),
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

float hfi_spd_ph_rep(void)
{
    return s_ph_rep;
}
#endif

#if M1_HFI_GATE == 133
/* 100 rpm 档的齿槽补偿。更高转速和 0 rpm 不加。 */
static float s_rip12_c;
static float s_rip12_s;
static float s_rip14_c;
static float s_rip14_s;
static float s_rip_amp;

void hfi_rip_cap(float *c, float *s)
{
    const float amp = 0.20f;
    float a = (*c) * (*c) + (*s) * (*s);

    if (a > (amp * amp)) {
        const float k = amp / motor_sqrt4(a);

        *c *= k;
        *s *= k;
    }
}

void hfi_rip_decay(void)
{
    s_rip12_c *= 0.90f;
    s_rip12_s *= 0.90f;
    s_rip14_c *= 0.90f;
    s_rip14_s *= 0.90f;
}

float hfi_ripple_iq(float iq, float w_ref)
{
    const float rpm_scale = 60.0f / (6.28318530718f * (float)OBS_POLE_PAIRS);
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
        const float p12 = motor_wrap_pi(th * (12.0f / (float)OBS_POLE_PAIRS));
        const float p14 = motor_wrap_pi(th * (14.0f / (float)OBS_POLE_PAIRS));
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
    s_rip_amp = motor_sqrt4((s_rip12_c * s_rip12_c) + (s_rip12_s * s_rip12_s)) +
                motor_sqrt4((s_rip14_c * s_rip14_c) + (s_rip14_s * s_rip14_s));
    if (iq > M1_SPEED_IQ_REF_ABS_MAX) {
        iq = M1_SPEED_IQ_REF_ABS_MAX;
    } else if (iq < -M1_SPEED_IQ_REF_ABS_MAX) {
        iq = -M1_SPEED_IQ_REF_ABS_MAX;
    }
    return iq;
}

float hfi_rip_amp(void)
{
    return s_rip_amp;
}
#endif
