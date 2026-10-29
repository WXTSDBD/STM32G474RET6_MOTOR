/**
 * @file emf_pll.c
 * @date 2026-10-06
 * @brief 反电势正交 Type-II PLL：sincos、PI、积分。热路径不用 atan2。
 *
 * 默认不进 Park。节拍限制见 emf_pll.h 文件头。
 *
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
 */

#include "emf_pll.h"

#include <math.h>
#include <stddef.h>

#include "observer/obs_cfg.h"
#include "obs_num.h"
#include "motor_trig.h"
#include "observer/obs_src.h"
#if M1_EMF_LPF_PHASE_FF_ENABLE && M1_EMF_SMO_ENABLE && M1_EMF_SMO_LPF_ENABLE
#include "emf_smo.h"
#endif

#if M1_EMF_PLL_ENABLE

#define EMF_PLL_PI       3.14159265358979323846f
#define EMF_PLL_TWO_PI   6.28318530717958647692f

/**
 * @brief 把角折回 (-π, π]。
 */
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

/**
 * @brief 按绝对值上限截断。
 */
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

/**
 * @brief atan2 多项式，与 VESC utils_fast_atan2 同一组系数。
 * @note 这里只用来算 atan(fe/fc)，fc 恒正。
 */
static float emf_pll_fast_atan2(float y, float x)
{
    float abs_y = (y >= 0.0f) ? y : -y;
    float angle;

    abs_y += 1.0e-20f;
    if (x >= 0.0f) {
        float r = (x - abs_y) / (x + abs_y);
        float rsq = r * r;

        angle = ((0.1963f * rsq) - 0.9817f) * r + 0.78539816f;
    } else {
        float r = (x + abs_y) / (abs_y - x);
        float rsq = r * r;

        angle = ((0.1963f * rsq) - 0.9817f) * r + 2.35619449f;
    }
    if (y < 0.0f) {
        angle = -angle;
    }
    return angle;
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
    return -emf_pll_fast_atan2(fe, fc);
#else
    (void)omega_el;
    return 0.0f;
#endif
}

/**
 * @brief 按自然频率和阻尼算 Kp/Ki，然后复位。
 * @param p PLL。不可为 NULL。
 */
void emf_pll_init(emf_pll_t *p)
{
    const float wn = EMF_PLL_TWO_PI * g_obs_num.pll_fn_hz;

    if (p == NULL) {
        return;
    }
    p->kp = 2.0f * g_obs_num.pll_zeta * wn;
    p->ki = wn * wn;
    p->omega_limit = g_obs_num.pll_omega_limit_rad_s;
    p->integrator_limit = g_obs_num.pll_integrator_limit_rad_s;
    emf_pll_reset(p);
}

/**
 * @brief 清角、ω 和积分。保留增益。
 * @param p PLL。不可为 NULL。
 */
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

/**
 * @brief 用 eαβ 推进 PLL。
 * @param p PLL。不可为 NULL。
 * @param e_alpha 反电势 α，单位 V。
 * @param e_beta 反电势 β，单位 V。
 * @param theta_enc 对照电角，单位 rad。
 * @param dt 节拍，单位 s。
 */
void emf_pll_update(emf_pll_t *p,
                    float e_alpha, float e_beta,
                    float theta_enc,
                    float dt)
{
    float cos_h;
    float sin_h;
    float err;
    float omega_cmd;

    if (p == NULL || dt <= 0.0f) {
        return;
    }

    if (p->primed == 0u) {
        /* 内部跟踪滤波后 e 帧；输出扣 θ_off + LPF 相位前馈 */
        p->theta = emf_pll_wrap_pi(theta_enc + g_obs_num.pll_theta_off_rad);
        p->omega_el = 0.0f;
        p->integrator = 0.0f;
        p->theta_hat = theta_enc;
        p->theta_err = 0.0f;
        p->last_pd = 0.0f;
        p->emag = 0.0f;
        p->primed = 1u;
        return;
    }

    motor_trig_sincos(p->theta, &cos_h, &sin_h);
    /* 正转：eα≈−|e|sinθ，eβ≈+|e|cosθ。反转反电动势反向。
     * 符号用高频注入转速（低速槽 get_omega，仍是 PLL 积分）。
     * 滑模自己的转速一旦估反，会把鉴相锁死。 */
    err = -e_alpha * cos_h - e_beta * sin_h;
    if (observer_lo_src()->get_omega() < 0.0f) {
        err = -err;
    }
#if M1_EMF_PLL_NORM_ENABLE
    {
        const float emag = sqrtf(e_alpha * e_alpha + e_beta * e_beta);

        /* 打开时与历史增益同口径：err/(|e|+eps)。 */
        p->emag = emag;
        err /= (emag + g_obs_num.pll_norm_eps);
    }
#else
    /* T1：鉴相用原始正交误差，省 sqrt+除法；增益等效随 |e| 变。 */
    p->emag = 0.0f;
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
        const float phi_ff = emf_pll_lpf_phase_ff(observer_lo_src()->get_omega());

        p->theta_hat = emf_pll_wrap_pi(
            p->theta - g_obs_num.pll_theta_off_rad - phi_ff);
    }
    p->theta_err = emf_pll_wrap_pi(p->theta_hat - theta_enc);
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

#endif /* M1_EMF_PLL_ENABLE */
