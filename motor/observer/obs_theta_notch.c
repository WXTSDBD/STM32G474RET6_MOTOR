/**
 * @file obs_theta_notch.c
 * @brief θ̂ 机械 1/rev 自适应陷波（纹波域 biquad）
 */

#include "obs_theta_notch.h"

#include <math.h>

#include "observer/obs_cfg.h"

#ifndef M1_OBS_THETA_NOTCH_ENABLE
#define M1_OBS_THETA_NOTCH_ENABLE       0
#endif

#if M1_OBS_THETA_NOTCH_ENABLE

#ifndef M1_OBS_THETA_NOTCH_Q
#define M1_OBS_THETA_NOTCH_Q            10.0f
#endif
#ifndef M1_OBS_THETA_NOTCH_TRACK_HZ
#define M1_OBS_THETA_NOTCH_TRACK_HZ     3.0f
#endif
#ifndef M1_OBS_THETA_NOTCH_RPM_MIN
#define M1_OBS_THETA_NOTCH_RPM_MIN      400.0f
#endif
#ifndef M1_OBS_THETA_NOTCH_H2_ENABLE
#define M1_OBS_THETA_NOTCH_H2_ENABLE    0
#endif
#ifndef M1_OBS_THETA_NOTCH_F0_SLEW_HZ_S
#define M1_OBS_THETA_NOTCH_F0_SLEW_HZ_S 80.0f
#endif
#ifndef M1_OBS_THETA_NOTCH_F0_UPDATE_HZ
#define M1_OBS_THETA_NOTCH_F0_UPDATE_HZ   0.25f
#endif

#define OBS_TH_PI       3.14159265358979323846f
#define OBS_TH_TWO_PI   6.28318530717958647692f

typedef struct {
    float b0, b1, b2, a1, a2; /* a0 已归一到 1 */
    float x1, x2, y1, y2;
    float f0_hz;
} obs_th_biquad_t;

static float s_track_k;
static float s_theta_lp;
static uint8_t s_lp_valid;
static float s_f0_cmd_hz;
static obs_th_biquad_t s_n1;
#if M1_OBS_THETA_NOTCH_H2_ENABLE
static obs_th_biquad_t s_n2;
#endif

static float obs_th_wrap_pi(float x)
{
    while (x > OBS_TH_PI) {
        x -= OBS_TH_TWO_PI;
    }
    while (x < -OBS_TH_PI) {
        x += OBS_TH_TWO_PI;
    }
    return x;
}

static float obs_th_clampf(float x, float lo, float hi)
{
    if (x < lo) {
        return lo;
    }
    if (x > hi) {
        return hi;
    }
    return x;
}

static void obs_th_biquad_clear(obs_th_biquad_t *b, float x0)
{
    b->x1 = x0;
    b->x2 = x0;
    b->y1 = x0;
    b->y2 = x0;
}

static void obs_th_biquad_set_notch(obs_th_biquad_t *b, float f0_hz, float dt)
{
    float w0;
    float cosw;
    float sinw;
    float alpha;
    float a0;
    float inv_a0;
    float fs;

    if (dt < 1.0e-6f) {
        return;
    }
    fs = 1.0f / dt;
    /* 防 Nyquist：f0 < 0.45*fs */
    f0_hz = obs_th_clampf(f0_hz, 0.5f, 0.45f * fs);
    w0 = OBS_TH_TWO_PI * f0_hz * dt;
    cosw = cosf(w0);
    sinw = sinf(w0);
    alpha = sinw / (2.0f * M1_OBS_THETA_NOTCH_Q);
    a0 = 1.0f + alpha;
    inv_a0 = 1.0f / a0;

    b->b0 = 1.0f * inv_a0;
    b->b1 = (-2.0f * cosw) * inv_a0;
    b->b2 = 1.0f * inv_a0;
    b->a1 = (-2.0f * cosw) * inv_a0;
    b->a2 = (1.0f - alpha) * inv_a0;
    b->f0_hz = f0_hz;
}

static float obs_th_biquad_step(obs_th_biquad_t *b, float x)
{
    float y = b->b0 * x + b->b1 * b->x1 + b->b2 * b->x2
              - b->a1 * b->y1 - b->a2 * b->y2;
    b->x2 = b->x1;
    b->x1 = x;
    b->y2 = b->y1;
    b->y1 = y;
    return y;
}

static void obs_th_update_f0(float f0_tgt, float dt)
{
    float df_max;
    float df;

    df_max = M1_OBS_THETA_NOTCH_F0_SLEW_HZ_S * dt;
    df = f0_tgt - s_f0_cmd_hz;
    if (df > df_max) {
        df = df_max;
    } else if (df < -df_max) {
        df = -df_max;
    }
    s_f0_cmd_hz += df;

    if (fabsf(s_f0_cmd_hz - s_n1.f0_hz) >= M1_OBS_THETA_NOTCH_F0_UPDATE_HZ) {
        obs_th_biquad_set_notch(&s_n1, s_f0_cmd_hz, dt);
#if M1_OBS_THETA_NOTCH_H2_ENABLE
        obs_th_biquad_set_notch(&s_n2, 2.0f * s_f0_cmd_hz, dt);
#endif
    }
}

void obs_theta_notch_init(void)
{
    s_track_k = 1.0f - expf(-OBS_TH_TWO_PI * M1_OBS_THETA_NOTCH_TRACK_HZ * OBS_CTRL_TS_S);
    if (s_track_k < 0.0f) {
        s_track_k = 0.0f;
    }
    if (s_track_k > 1.0f) {
        s_track_k = 1.0f;
    }
    obs_theta_notch_reset();
}

void obs_theta_notch_reset(void)
{
    s_theta_lp = 0.0f;
    s_lp_valid = 0u;
    s_f0_cmd_hz = 16.6667f; /* ~1000 rpm 初值，随后自适应 */
    obs_th_biquad_set_notch(&s_n1, s_f0_cmd_hz, OBS_CTRL_TS_S);
    obs_th_biquad_clear(&s_n1, 0.0f);
#if M1_OBS_THETA_NOTCH_H2_ENABLE
    obs_th_biquad_set_notch(&s_n2, 2.0f * s_f0_cmd_hz, OBS_CTRL_TS_S);
    obs_th_biquad_clear(&s_n2, 0.0f);
#endif
}

float obs_theta_notch_apply(float theta_hat,
                            float omega_obs_rpm,
                            uint8_t enable,
                            float dt)
{
    float e;
    float e_n;
    float rpm_abs;
    float f0_tgt;
    uint8_t active;

    if (s_lp_valid == 0u) {
        s_theta_lp = theta_hat;
        s_lp_valid = 1u;
        obs_th_biquad_clear(&s_n1, 0.0f);
#if M1_OBS_THETA_NOTCH_H2_ENABLE
        obs_th_biquad_clear(&s_n2, 0.0f);
#endif
        return theta_hat;
    }

    /* 慢跟踪抽纹波；输出 = θ̂ + (e_n−e)，全通时 e_n=e → 原样 */
    e = obs_th_wrap_pi(theta_hat - s_theta_lp);
    s_theta_lp = obs_th_wrap_pi(s_theta_lp + s_track_k * e);

    rpm_abs = fabsf(omega_obs_rpm);
    active = (enable != 0u) && (rpm_abs >= M1_OBS_THETA_NOTCH_RPM_MIN) ? 1u : 0u;
    if (active == 0u) {
        obs_th_biquad_clear(&s_n1, e);
#if M1_OBS_THETA_NOTCH_H2_ENABLE
        obs_th_biquad_clear(&s_n2, e);
#endif
        return theta_hat;
    }

    f0_tgt = rpm_abs / 60.0f;
    obs_th_update_f0(f0_tgt, dt);

    e_n = obs_th_biquad_step(&s_n1, e);
#if M1_OBS_THETA_NOTCH_H2_ENABLE
    e_n = obs_th_biquad_step(&s_n2, e_n);
#endif

    return obs_th_wrap_pi(theta_hat + (e_n - e));
}

#else /* !M1_OBS_THETA_NOTCH_ENABLE */

void obs_theta_notch_init(void)
{
}

void obs_theta_notch_reset(void)
{
}

float obs_theta_notch_apply(float theta_hat,
                            float omega_obs_rpm,
                            uint8_t enable,
                            float dt)
{
    (void)omega_obs_rpm;
    (void)enable;
    (void)dt;
    return theta_hat;
}

#endif /* M1_OBS_THETA_NOTCH_ENABLE */
