/**
 * @file deadband.c
 */

#include "deadband.h"

#include <math.h>
#include <stddef.h>

#include "motor_params_m1.h"

static m1_deadband_cfg_t s_cfg;

static float deadband_phase_sign(float i, float i_zero)
{
    if (i > i_zero) {
        return 1.0f;
    }
    if (i < -i_zero) {
        return -1.0f;
    }
    return 0.0f;
}

static float deadband_lut_lookup_v(float i_abs)
{
    const float *amps = s_cfg.lut_amps;
    const float *vals = s_cfg.lut_vals;
    uint8_t n = s_cfg.lut_len;
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

static float deadband_comp_mag_v(float i_abs)
{
    if (s_cfg.mode == M1_DEADBAND_MODE_LUT) {
        return deadband_lut_lookup_v(i_abs);
    }
    return s_cfg.v_comp_v;
}

static void deadband_clamp_duty(float *d)
{
    if (*d < 0.0f) {
        *d = 0.0f;
    } else if (*d > 1.0f) {
        *d = 1.0f;
    }
}

void deadband_init(void)
{
#if M1_DEADBAND_ENABLE
    s_cfg.mode = M1_DEADBAND_MODE_FIXED;
#else
    s_cfg.mode = M1_DEADBAND_MODE_OFF;
#endif
    s_cfg.v_comp_v = M1_DEADBAND_V_COMP_V;
    s_cfg.i_zero_a = M1_DEADBAND_I_ZERO_A;
    s_cfg.lut_amps = NULL;
    s_cfg.lut_vals = NULL;
    s_cfg.lut_len = 0u;
}

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

void deadband_set_lut(const float *amps, const float *vals, uint8_t len)
{
    if (amps == NULL || vals == NULL || len < 2u || len > M1_DEADBAND_LUT_MAX) {
        return;
    }

    s_cfg.lut_amps = amps;
    s_cfg.lut_vals = vals;
    s_cfg.lut_len = len;
    s_cfg.mode = M1_DEADBAND_MODE_LUT;
}

float deadband_comp_v(float i_a)
{
    float i_abs;
    float mag_v;
    float sign_i;

    if (s_cfg.mode == M1_DEADBAND_MODE_OFF) {
        return 0.0f;
    }

    sign_i = deadband_phase_sign(i_a, s_cfg.i_zero_a);
    if (sign_i == 0.0f) {
        return 0.0f;
    }

    i_abs = (i_a >= 0.0f) ? i_a : -i_a;
    mag_v = deadband_comp_mag_v(i_abs);
    return mag_v * sign_i;
}

void deadband_apply_duty(float ia, float ib, float ic,
                         float *duty_a, float *duty_b, float *duty_c)
{
    float duty_comp;

    if (s_cfg.mode == M1_DEADBAND_MODE_OFF) {
        return;
    }

    duty_comp = M1_DEADBAND_DUTY_COMP;

    if (s_cfg.mode == M1_DEADBAND_MODE_FIXED) {
        *duty_a += duty_comp * deadband_phase_sign(ia, s_cfg.i_zero_a);
        *duty_b += duty_comp * deadband_phase_sign(ib, s_cfg.i_zero_a);
        *duty_c += duty_comp * deadband_phase_sign(ic, s_cfg.i_zero_a);
    } else {
        *duty_a += deadband_comp_v(ia) / M1_VBUS_V;
        *duty_b += deadband_comp_v(ib) / M1_VBUS_V;
        *duty_c += deadband_comp_v(ic) / M1_VBUS_V;
    }

    deadband_clamp_duty(duty_a);
    deadband_clamp_duty(duty_b);
    deadband_clamp_duty(duty_c);
}
