/**
 * @file emf_smo.c
 * @brief Classic SMO — ISR: mul/add + atan2; LPF α：固定 / 分档表 / 线性 fc(n)。
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
#ifndef M1_EMF_SMO_LPF_SCHED_ENABLE
#define M1_EMF_SMO_LPF_SCHED_ENABLE 0
#endif
#ifndef M1_EMF_SMO_LPF_LINEAR_ENABLE
#define M1_EMF_SMO_LPF_LINEAR_ENABLE 0
#endif
#if M1_EMF_SMO_LPF_LINEAR_ENABLE
#ifndef M1_EMF_SMO_LPF_LINEAR_K
#define M1_EMF_SMO_LPF_LINEAR_K     1.5f
#endif
#ifndef M1_EMF_SMO_LPF_LINEAR_FC_MIN
#define M1_EMF_SMO_LPF_LINEAR_FC_MIN 100.0f
#endif
#ifndef M1_EMF_SMO_LPF_LINEAR_FC_MAX
#define M1_EMF_SMO_LPF_LINEAR_FC_MAX 280.0f
#endif
#endif

#define EMF_SMO_PI       3.14159265358979323846f
#define EMF_SMO_TWO_PI   6.28318530717958647692f

#if M1_EMF_SMO_LPF_SCHED_ENABLE && !M1_EMF_SMO_LPF_LINEAR_ENABLE
#ifndef M1_EMF_SMO_LPF_SCHED_BANDS
#define M1_EMF_SMO_LPF_SCHED_BANDS  7u
#endif
#if M1_EMF_SMO_LPF_SCHED_BANDS != 7u
#error "emf_smo LPF sched table sized for 7 bands"
#endif
#ifndef M1_EMF_SMO_LPF_UP1
#define M1_EMF_SMO_LPF_UP1   500.0f
#define M1_EMF_SMO_LPF_UP2   650.0f
#define M1_EMF_SMO_LPF_UP3   750.0f
#define M1_EMF_SMO_LPF_UP4   850.0f
#define M1_EMF_SMO_LPF_UP5  1050.0f
#define M1_EMF_SMO_LPF_UP6  1150.0f
#endif
#ifndef M1_EMF_SMO_LPF_FC0
#define M1_EMF_SMO_LPF_FC0   100.0f
#define M1_EMF_SMO_LPF_FC1   120.0f
#define M1_EMF_SMO_LPF_FC2   145.0f
#define M1_EMF_SMO_LPF_FC3   170.0f
#define M1_EMF_SMO_LPF_FC4   200.0f
#define M1_EMF_SMO_LPF_FC5   220.0f
#define M1_EMF_SMO_LPF_FC6   240.0f
#endif
#endif /* band sched */

static float s_disc_a;
static float s_disc_b;
static float s_lpf_alpha;
static float s_lpf_hz;
static float s_lpf_ts;
static float s_rpm_to_we;
static float s_inv_sat;
static float s_rpm_to_fe; /* |rpm| → fe [Hz] = rpm * pp / 60 */
static uint8_t s_coeff_ready;
#if M1_EMF_SMO_LPF_SCHED_ENABLE && !M1_EMF_SMO_LPF_LINEAR_ENABLE
static float s_lpf_alpha_tab[7];
static float s_lpf_hz_tab[7];
static uint8_t s_lpf_band;
static uint8_t s_lpf_tab_ready;
#endif

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

static float emf_smo_alpha_from_hz(float fc_hz, float ts)
{
    float a = 1.0f - expf(-EMF_SMO_TWO_PI * fc_hz * ts);
    if (a > 1.0f) {
        a = 1.0f;
    }
    if (a < 0.0f) {
        a = 0.0f;
    }
    return a;
}

#if M1_EMF_SMO_LPF_SCHED_ENABLE && !M1_EMF_SMO_LPF_LINEAR_ENABLE
static void emf_smo_lpf_tab_init(float ts)
{
    const float fc[7] = {
        M1_EMF_SMO_LPF_FC0, M1_EMF_SMO_LPF_FC1, M1_EMF_SMO_LPF_FC2,
        M1_EMF_SMO_LPF_FC3, M1_EMF_SMO_LPF_FC4, M1_EMF_SMO_LPF_FC5,
        M1_EMF_SMO_LPF_FC6
    };
    uint8_t i;

    for (i = 0u; i < 7u; i++) {
        s_lpf_hz_tab[i] = fc[i];
        s_lpf_alpha_tab[i] = emf_smo_alpha_from_hz(fc[i], ts);
    }
    s_lpf_band = 0u;
    s_lpf_alpha = s_lpf_alpha_tab[0];
    s_lpf_hz = s_lpf_hz_tab[0];
    s_lpf_tab_ready = 1u;
}
#endif

static void emf_smo_coeff_init(void)
{
    const float ts = M1_CTRL_TS_S;
    const float r = M1_EMF_SMO_R_OHM;
    const float l = M1_EMF_SMO_L_H;

    s_disc_a = expf(-r * ts / l);
    s_disc_b = (1.0f - s_disc_a) / r;
    s_lpf_ts = ts;
#if M1_EMF_SMO_LPF_ENABLE
#if M1_EMF_SMO_LPF_LINEAR_ENABLE
    s_lpf_hz = M1_EMF_SMO_LPF_LINEAR_FC_MIN;
    s_lpf_alpha = emf_smo_alpha_from_hz(s_lpf_hz, ts);
#elif M1_EMF_SMO_LPF_SCHED_ENABLE
    emf_smo_lpf_tab_init(ts);
#else
    s_lpf_hz = M1_EMF_SMO_LPF_HZ;
    s_lpf_alpha = emf_smo_alpha_from_hz(M1_EMF_SMO_LPF_HZ, ts);
#endif
#else
    s_lpf_hz = 0.0f;
    s_lpf_alpha = 1.0f;
#endif
    s_rpm_to_we = (EMF_SMO_TWO_PI / 60.0f) * (float)M1_POLE_PAIRS;
    s_rpm_to_fe = ((float)M1_POLE_PAIRS) / 60.0f;
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
#if M1_EMF_SMO_LPF_LINEAR_ENABLE
    s_lpf_hz = M1_EMF_SMO_LPF_LINEAR_FC_MIN;
    s_lpf_alpha = emf_smo_alpha_from_hz(s_lpf_hz, s_lpf_ts);
#elif M1_EMF_SMO_LPF_SCHED_ENABLE
    if (s_lpf_tab_ready != 0u) {
        s_lpf_band = 0u;
        s_lpf_alpha = s_lpf_alpha_tab[0];
        s_lpf_hz = s_lpf_hz_tab[0];
    }
#endif
}

uint8_t emf_smo_lpf_sched_update(float omega_mech_rpm)
{
#if M1_EMF_SMO_LPF_ENABLE && M1_EMF_SMO_LPF_LINEAR_ENABLE
    float rpm;
    float fe;
    float fc;
    float fc_q;

    if (s_coeff_ready == 0u) {
        emf_smo_coeff_init();
    }

    rpm = omega_mech_rpm;
    if (rpm < 0.0f) {
        rpm = -rpm;
    }
    /* fc = clip(k * fe, min, max), fe = |n| * p / 60 */
    fe = rpm * s_rpm_to_fe;
    fc = M1_EMF_SMO_LPF_LINEAR_K * fe;
    if (fc < M1_EMF_SMO_LPF_LINEAR_FC_MIN) {
        fc = M1_EMF_SMO_LPF_LINEAR_FC_MIN;
    } else if (fc > M1_EMF_SMO_LPF_LINEAR_FC_MAX) {
        fc = M1_EMF_SMO_LPF_LINEAR_FC_MAX;
    }
    /* 量化到 1 Hz，避免每拍 expf */
    fc_q = (float)((int)(fc + 0.5f));
    if (fc_q != s_lpf_hz) {
        s_lpf_hz = fc_q;
        s_lpf_alpha = emf_smo_alpha_from_hz(fc_q, s_lpf_ts);
    }
    return 0u;
#elif M1_EMF_SMO_LPF_SCHED_ENABLE && M1_EMF_SMO_LPF_ENABLE
    float rpm;
    uint8_t band;

    if (s_coeff_ready == 0u) {
        emf_smo_coeff_init();
    }
    if (s_lpf_tab_ready == 0u) {
        return 0u;
    }

    rpm = omega_mech_rpm;
    if (rpm < 0.0f) {
        rpm = -rpm;
    }

    if (rpm >= M1_EMF_SMO_LPF_UP6) {
        band = 6u;
    } else if (rpm >= M1_EMF_SMO_LPF_UP5) {
        band = 5u;
    } else if (rpm >= M1_EMF_SMO_LPF_UP4) {
        band = 4u;
    } else if (rpm >= M1_EMF_SMO_LPF_UP3) {
        band = 3u;
    } else if (rpm >= M1_EMF_SMO_LPF_UP2) {
        band = 2u;
    } else if (rpm >= M1_EMF_SMO_LPF_UP1) {
        band = 1u;
    } else {
        band = 0u;
    }

    if (band != s_lpf_band) {
        s_lpf_band = band;
        s_lpf_alpha = s_lpf_alpha_tab[band];
        s_lpf_hz = s_lpf_hz_tab[band];
    }
    return s_lpf_band;
#else
    (void)omega_mech_rpm;
    return 0u;
#endif
}

float emf_smo_get_lpf_hz(void)
{
#if M1_EMF_SMO_ENABLE
    if (s_coeff_ready == 0u) {
        emf_smo_coeff_init();
    }
    return s_lpf_hz;
#else
    return 0.0f;
#endif
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

uint8_t emf_smo_lpf_sched_update(float omega_mech_rpm)
{
    (void)omega_mech_rpm;
    return 0u;
}

float emf_smo_get_lpf_hz(void)
{
    return 0.0f;
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
