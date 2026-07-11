/**
 * @file foc_pi.c
 */

#include <stddef.h>
#include <stdint.h>

#include "foc_pi.h"

static void foc_pi_clamp_integrator(foc_pi_t *pi)
{
    if (pi->integrator > pi->int_max) {
        pi->integrator = pi->int_max;
    } else if (pi->integrator < pi->int_min) {
        pi->integrator = pi->int_min;
    }
}

void foc_pi_init(foc_pi_t *pi, float kp, float ki,
                 float out_min, float out_max,
                 float int_min, float int_max)
{
    if (pi == NULL) {
        return;
    }

    pi->kp = kp;
    pi->ki = ki;
    pi->integrator = 0.0f;
    pi->out_min = out_min;
    pi->out_max = out_max;
    pi->int_min = int_min;
    pi->int_max = int_max;
}

void foc_pi_reset(foc_pi_t *pi)
{
    if (pi == NULL) {
        return;
    }

    pi->integrator = 0.0f;
}

void foc_pi_bumpless(foc_pi_t *pi, float u_prev, float ref, float fb)
{
    foc_pi_bumpless_beta(pi, u_prev, ref, fb, 1.0f);
}

float foc_pi_step(foc_pi_t *pi, float ref, float fb)
{
    return foc_pi_step_beta(pi, ref, fb, 1.0f);
}

float foc_pi_step_beta(foc_pi_t *pi, float ref, float fb, float beta)
{
    float err;
    float out;
    float p_term;

    if (pi == NULL) {
        return 0.0f;
    }

    err = ref - fb;
    pi->integrator += pi->ki * err;
    foc_pi_clamp_integrator(pi);
    p_term = pi->kp * (beta * ref - fb);
    out = p_term + pi->integrator;

    if (out > pi->out_max) {
        out = pi->out_max;
        pi->integrator = pi->out_max - p_term;
        foc_pi_clamp_integrator(pi);
    } else if (out < pi->out_min) {
        out = pi->out_min;
        pi->integrator = pi->out_min - p_term;
        foc_pi_clamp_integrator(pi);
    }

    return out;
}

float foc_pi_step_beta_hold_i(foc_pi_t *pi, float ref, float fb, float beta,
                              uint8_t hold_i)
{
    float err;
    float out;
    float p_term;

    if (pi == NULL) {
        return 0.0f;
    }

    err = ref - fb;
    if (hold_i == 0u) {
        pi->integrator += pi->ki * err;
        foc_pi_clamp_integrator(pi);
    }
    p_term = pi->kp * (beta * ref - fb);
    out = p_term + pi->integrator;

    if (out > pi->out_max) {
        out = pi->out_max;
        pi->integrator = pi->out_max - p_term;
        foc_pi_clamp_integrator(pi);
    } else if (out < pi->out_min) {
        out = pi->out_min;
        pi->integrator = pi->out_min - p_term;
        foc_pi_clamp_integrator(pi);
    }

    return out;
}

void foc_pi_bumpless_beta(foc_pi_t *pi, float u_prev, float ref, float fb, float beta)
{
    if (pi == NULL) {
        return;
    }

    pi->integrator = u_prev - pi->kp * (beta * ref - fb);
    foc_pi_clamp_integrator(pi);
}
