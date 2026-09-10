/**
 * @file emf_smo.c
 * @brief Classic SMO — ISR: mul/add + atan2; a/b/LPF α at init (no expf in tick).
 */

#include "emf_smo.h"

#include <math.h>
#include <stddef.h>

#include "motor_params_m1.h"

#ifndef M1_EMF_SMO_ENABLE
#define M1_EMF_SMO_ENABLE 0
#endif

#if M1_EMF_SMO_ENABLE

#ifndef M1_EMF_SMO_R_OHM
#define M1_EMF_SMO_R_OHM            M1_RS_OHM
#endif
#ifndef M1_EMF_SMO_L_H
#define M1_EMF_SMO_L_H              M1_LD_H
#endif
#ifndef M1_EMF_SMO_K
#define M1_EMF_SMO_K                20.0f
#endif
#ifndef M1_EMF_SMO_SAT_A
#define M1_EMF_SMO_SAT_A            0.30f
#endif
#ifndef M1_EMF_SMO_LPF_HZ
#define M1_EMF_SMO_LPF_HZ           200.0f
#endif
#ifndef M1_EMF_SMO_LPF_ENABLE
#define M1_EMF_SMO_LPF_ENABLE       1
#endif
#ifndef M1_EMF_SMO_THETA_OFF_RAD
#define M1_EMF_SMO_THETA_OFF_RAD    (-0.4054f)
#endif

#define EMF_SMO_PI       3.14159265358979323846f
#define EMF_SMO_TWO_PI   6.28318530717958647692f

static float s_disc_a;
static float s_disc_b;
static float s_lpf_alpha;
static float s_rpm_to_we;
static float s_inv_sat;
static uint8_t s_coeff_ready;

static float emf_smo_wrap_pi(float x)
{
    if (x > EMF_SMO_PI) {
        x -= EMF_SMO_TWO_PI;
    } else if (x < -EMF_SMO_PI) {
        x += EMF_SMO_TWO_PI;
    }
    return x;
}

static float emf_smo_sat_fast(float x)
{
    x *= s_inv_sat;
    if (x > 1.0f) {
        return 1.0f;
    }
    if (x < -1.0f) {
        return -1.0f;
    }
    return x;
}

static void emf_smo_coeff_init(void)
{
    const float ts = M1_CTRL_TS_S;
    const float r = M1_EMF_SMO_R_OHM;
    const float l = M1_EMF_SMO_L_H;

    s_disc_a = expf(-r * ts / l);
    s_disc_b = (1.0f - s_disc_a) / r;
#if M1_EMF_SMO_LPF_ENABLE
    s_lpf_alpha = 1.0f - expf(-EMF_SMO_TWO_PI * M1_EMF_SMO_LPF_HZ * ts);
    if (s_lpf_alpha > 1.0f) {
        s_lpf_alpha = 1.0f;
    }
    if (s_lpf_alpha < 0.0f) {
        s_lpf_alpha = 0.0f;
    }
#else
    s_lpf_alpha = 1.0f;
#endif
    s_rpm_to_we = (EMF_SMO_TWO_PI / 60.0f) * (float)M1_POLE_PAIRS;
    s_inv_sat = 1.0f / ((M1_EMF_SMO_SAT_A > 1.0e-6f) ? M1_EMF_SMO_SAT_A : 1.0e-6f);
    s_coeff_ready = 1u;
}

void emf_smo_init(emf_smo_t *o)
{
    if (s_coeff_ready == 0u) {
        emf_smo_coeff_init();
    }
    emf_smo_reset(o);
}

void emf_smo_reset(emf_smo_t *o)
{
    if (o == NULL) {
        return;
    }
    o->ihat_alpha = 0.0f;
    o->ihat_beta = 0.0f;
    o->primed = 0u;
    o->u_alpha = 0.0f;
    o->u_beta = 0.0f;
    o->e_alpha = 0.0f;
    o->e_beta = 0.0f;
    o->emag = 0.0f;
    o->theta_hat = 0.0f;
    o->theta_err = 0.0f;
    o->omega_el = 0.0f;
}

void emf_smo_update(emf_smo_t *o,
                    float i_alpha, float i_beta,
                    float u_alpha, float u_beta,
                    float theta_enc,
                    float omega_mech_rpm)
{
    float za;
    float zb;
    float th_hat;
    const float k = M1_EMF_SMO_K;

    if (o == NULL) {
        return;
    }
    if (s_coeff_ready == 0u) {
        emf_smo_coeff_init();
    }

    o->u_alpha = u_alpha;
    o->u_beta = u_beta;

    if (o->primed == 0u) {
        o->ihat_alpha = i_alpha;
        o->ihat_beta = i_beta;
        o->primed = 1u;
        o->e_alpha = 0.0f;
        o->e_beta = 0.0f;
        o->emag = 0.0f;
        o->theta_hat = theta_enc;
        o->theta_err = 0.0f;
        o->omega_el = omega_mech_rpm * s_rpm_to_we;
        return;
    }

    za = k * emf_smo_sat_fast(o->ihat_alpha - i_alpha);
    zb = k * emf_smo_sat_fast(o->ihat_beta - i_beta);

    o->ihat_alpha = s_disc_a * o->ihat_alpha + s_disc_b * (u_alpha - za);
    o->ihat_beta = s_disc_a * o->ihat_beta + s_disc_b * (u_beta - zb);

#if M1_EMF_SMO_LPF_ENABLE
    o->e_alpha += s_lpf_alpha * (za - o->e_alpha);
    o->e_beta += s_lpf_alpha * (zb - o->e_beta);
#else
    o->e_alpha = za;
    o->e_beta = zb;
#endif

    o->emag = sqrtf(o->e_alpha * o->e_alpha + o->e_beta * o->e_beta);
    th_hat = atan2f(-o->e_alpha, o->e_beta) - M1_EMF_SMO_THETA_OFF_RAD;
    o->theta_hat = emf_smo_wrap_pi(th_hat);
    o->theta_err = emf_smo_wrap_pi(o->theta_hat - theta_enc);
    o->omega_el = omega_mech_rpm * s_rpm_to_we;
}

#else /* !M1_EMF_SMO_ENABLE */

void emf_smo_init(emf_smo_t *o)
{
    (void)o;
}

void emf_smo_reset(emf_smo_t *o)
{
    (void)o;
}

void emf_smo_update(emf_smo_t *o,
                    float i_alpha, float i_beta,
                    float u_alpha, float u_beta,
                    float theta_enc,
                    float omega_mech_rpm)
{
    (void)o;
    (void)i_alpha;
    (void)i_beta;
    (void)u_alpha;
    (void)u_beta;
    (void)theta_enc;
    (void)omega_mech_rpm;
}

#endif /* M1_EMF_SMO_ENABLE */
