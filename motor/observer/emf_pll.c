/**
 * @file emf_pll.c
 * @brief EMF 正交 Type-II PLL — ISR: sincos + PI + 积分（无 atan2）
 */

#include "emf_pll.h"

#include <math.h>
#include <stddef.h>

#include "motor_params_m1.h"
#include "motor_trig.h"
#include "hfi_sqwave.h"
#if M1_EMF_LPF_PHASE_FF_ENABLE && M1_EMF_SMO_ENABLE && M1_EMF_SMO_LPF_ENABLE
#include "emf_smo.h"
#endif

#ifndef M1_EMF_PLL_ENABLE
#define M1_EMF_PLL_ENABLE 0
#endif

#if M1_EMF_PLL_ENABLE

#ifndef M1_EMF_PLL_FN_HZ
#define M1_EMF_PLL_FN_HZ                100.0f
#endif
#ifndef M1_EMF_PLL_ZETA
#define M1_EMF_PLL_ZETA                 0.707106781f
#endif
#ifndef M1_EMF_PLL_THETA_OFF_RAD
#define M1_EMF_PLL_THETA_OFF_RAD        (-0.4054f)
#endif
#ifndef M1_EMF_LPF_PHASE_FF_ENABLE
#define M1_EMF_LPF_PHASE_FF_ENABLE      0
#endif
#ifndef M1_EMF_PLL_NORM_ENABLE
#define M1_EMF_PLL_NORM_ENABLE          1
#endif
#ifndef M1_EMF_PLL_NORM_EPS
#define M1_EMF_PLL_NORM_EPS             0.05f
#endif
#ifndef M1_EMF_PLL_OMEGA_LIMIT_RAD_S
#define M1_EMF_PLL_OMEGA_LIMIT_RAD_S    2000.0f
#endif
#ifndef M1_EMF_PLL_INTEGRATOR_LIMIT_RAD_S
#define M1_EMF_PLL_INTEGRATOR_LIMIT_RAD_S M1_EMF_PLL_OMEGA_LIMIT_RAD_S
#endif

#define EMF_PLL_PI       3.14159265358979323846f
#define EMF_PLL_TWO_PI   6.28318530717958647692f

static float emf_pll_wrap_pi(float x)
{
    while (x > EMF_PLL_PI) {
        x -= EMF_PLL_TWO_PI;
    }
    while (x < -EMF_PLL_PI) {
        x += EMF_PLL_TWO_PI;
    }
    return x;
}

static float emf_pll_clamp(float x, float lim)
{
    if (x > lim) {
        return lim;
    }
    if (x < -lim) {
        return -lim;
    }
    return x;
}

/** 低通滞后。正转提前 θ̂，反转改为滞后。返回值从 θ̂ 里减去。 */
static float emf_pll_lpf_phase_ff(float omega_el)
{
#if M1_EMF_LPF_PHASE_FF_ENABLE && M1_EMF_SMO_ENABLE && M1_EMF_SMO_LPF_ENABLE
    float fe;
    float fc;
    float w;

    fe = omega_el / EMF_PLL_TWO_PI;
    fc = emf_smo_get_lpf_hz();
    w = fe;
    if (w < 0.0f) {
        w = -w;
    }
    if (w < 1.0f || fc < 1.0f) {
        return 0.0f;
    }
    return -atanf(fe / fc);
#else
    (void)omega_el;
    return 0.0f;
#endif
}

void emf_pll_init(emf_pll_t *p)
{
    const float wn = EMF_PLL_TWO_PI * M1_EMF_PLL_FN_HZ;

    if (p == NULL) {
        return;
    }
    p->kp = 2.0f * M1_EMF_PLL_ZETA * wn;
    p->ki = wn * wn;
    p->omega_limit = M1_EMF_PLL_OMEGA_LIMIT_RAD_S;
    p->integrator_limit = M1_EMF_PLL_INTEGRATOR_LIMIT_RAD_S;
    emf_pll_reset(p);
}

void emf_pll_reset(emf_pll_t *p)
{
    if (p == NULL) {
        return;
    }
    p->theta = 0.0f;
    p->omega_el = 0.0f;
    p->integrator = 0.0f;
    p->last_pd = 0.0f;
    p->emag = 0.0f;
    p->theta_hat = 0.0f;
    p->theta_err = 0.0f;
    p->primed = 0u;
}

void emf_pll_update(emf_pll_t *p,
                    float e_alpha, float e_beta,
                    float theta_enc,
                    float dt)
{
    float cos_h;
    float sin_h;
    float err;
    float omega_cmd;
    float emag;

    if (p == NULL || dt <= 0.0f) {
        return;
    }

    emag = sqrtf(e_alpha * e_alpha + e_beta * e_beta);
    p->emag = emag;

    if (p->primed == 0u) {
        /* 内部跟踪滤波后 e 帧；输出扣 θ_off + LPF 相位前馈 */
        p->theta = emf_pll_wrap_pi(theta_enc + M1_EMF_PLL_THETA_OFF_RAD);
        p->omega_el = 0.0f;
        p->integrator = 0.0f;
        p->theta_hat = theta_enc;
        p->theta_err = 0.0f;
        p->last_pd = 0.0f;
        p->primed = 1u;
        return;
    }

    motor_trig_sincos(p->theta, &cos_h, &sin_h);
    /* 正转：eα≈−|e|sinθ，eβ≈+|e|cosθ。反转反电动势反向。
     * 符号用高频注入转速。滑模自己的转速一旦估反，会把鉴相锁死。 */
    err = -e_alpha * cos_h - e_beta * sin_h;
    if (hfi_sqwave_get_pll_int_el() < 0.0f) {
        err = -err;
    }
#if M1_EMF_PLL_NORM_ENABLE
    err /= (emag + M1_EMF_PLL_NORM_EPS);
#endif
    p->last_pd = err;

    p->integrator += p->ki * err * dt;
    p->integrator = emf_pll_clamp(p->integrator, p->integrator_limit);

    omega_cmd = p->integrator + p->kp * err;
    if (omega_cmd > p->omega_limit) {
        p->omega_el = p->omega_limit;
        p->integrator = p->omega_limit - p->kp * err;
        p->integrator = emf_pll_clamp(p->integrator, p->integrator_limit);
    } else if (omega_cmd < -p->omega_limit) {
        p->omega_el = -p->omega_limit;
        p->integrator = -p->omega_limit - p->kp * err;
        p->integrator = emf_pll_clamp(p->integrator, p->integrator_limit);
    } else {
        p->omega_el = omega_cmd;
    }

    p->theta = emf_pll_wrap_pi(p->theta + p->omega_el * dt);
    {
        const float phi_ff = emf_pll_lpf_phase_ff(hfi_sqwave_get_pll_int_el());

        p->theta_hat = emf_pll_wrap_pi(
            p->theta - M1_EMF_PLL_THETA_OFF_RAD - phi_ff);
    }
    p->theta_err = emf_pll_wrap_pi(p->theta_hat - theta_enc);
}

float emf_pll_theta_smooth(const emf_pll_t *p, float omega_slow_el)
{
    float phi_slow;

    if (p == NULL) {
        return 0.0f;
    }
    phi_slow = emf_pll_lpf_phase_ff(omega_slow_el);
    return emf_pll_wrap_pi(p->theta - M1_EMF_PLL_THETA_OFF_RAD - phi_slow);
}

#else /* !M1_EMF_PLL_ENABLE */

void emf_pll_init(emf_pll_t *p)
{
    (void)p;
}

void emf_pll_reset(emf_pll_t *p)
{
    (void)p;
}

void emf_pll_update(emf_pll_t *p,
                    float e_alpha, float e_beta,
                    float theta_enc,
                    float dt)
{
    (void)p;
    (void)e_alpha;
    (void)e_beta;
    (void)theta_enc;
    (void)dt;
}

float emf_pll_theta_smooth(const emf_pll_t *p, float omega_slow_el)
{
    (void)p;
    (void)omega_slow_el;
    return 0.0f;
}

#endif /* M1_EMF_PLL_ENABLE */
