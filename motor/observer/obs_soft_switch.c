/**
 * @file obs_soft_switch.c
 * @brief 软切状态机：门限武装 → α 融合 → 观测角；可选推迟切速。
 */

#include "obs_soft_switch.h"

#include "observer/obs_cfg.h"

#ifndef M1_OBS_SOFT_SWITCH_ENABLE
#define M1_OBS_SOFT_SWITCH_ENABLE 0
#endif

#if M1_OBS_SOFT_SWITCH_ENABLE

#ifndef M1_OBS_SS_RPM_ENTER
#define M1_OBS_SS_RPM_ENTER             900.0f
#endif
#ifndef M1_OBS_SS_RPM_EXIT
#define M1_OBS_SS_RPM_EXIT              750.0f
#endif
#ifndef M1_OBS_SS_ERR_ENTER_RAD
#define M1_OBS_SS_ERR_ENTER_RAD         0.2617994f /* 15° */
#endif
#ifndef M1_OBS_SS_ERR_EXIT_RAD
#define M1_OBS_SS_ERR_EXIT_RAD          0.5235988f /* 30° */
#endif
#ifndef M1_OBS_SS_EMAG_MIN
#define M1_OBS_SS_EMAG_MIN              1.0f
#endif
#ifndef M1_OBS_SS_IQ_ABS_MAX
#define M1_OBS_SS_IQ_ABS_MAX            8.0f
#endif
#ifndef M1_OBS_SS_ARM_S
#define M1_OBS_SS_ARM_S                 1.0f
#endif
#ifndef M1_OBS_SS_ARM_GRACE_S
#define M1_OBS_SS_ARM_GRACE_S           0.05f
#endif
#ifndef M1_OBS_SS_BLEND_S
#define M1_OBS_SS_BLEND_S               0.15f
#endif
#ifndef M1_OBS_SS_RPM_TRIP_GRACE_S
#define M1_OBS_SS_RPM_TRIP_GRACE_S      0.05f /* 掉速回退防抖 */
#endif
#ifndef M1_OBS_SS_FALLBACK_ON_ERR_ENABLE
#define M1_OBS_SS_FALLBACK_ON_ERR_ENABLE 1
#endif
#ifndef M1_OBS_SS_FALLBACK_ENABLE
#define M1_OBS_SS_FALLBACK_ENABLE       1
#endif
#ifndef M1_OBS_SS_SPEED_SWITCH_ENABLE
#define M1_OBS_SS_SPEED_SWITCH_ENABLE   0
#endif
#ifndef M1_OBS_SS_SPD_DEFER_ENABLE
#define M1_OBS_SS_SPD_DEFER_ENABLE      0
#endif
#ifndef M1_OBS_SS_SPD_DWELL_S
#define M1_OBS_SS_SPD_DWELL_S           1.5f
#endif
#ifndef M1_OBS_SS_SPD_HOLD_S
#define M1_OBS_SS_SPD_HOLD_S            0.4f
#endif
#ifndef M1_OBS_SS_SPD_RPM_ERR_FRAC
#define M1_OBS_SS_SPD_RPM_ERR_FRAC      0.08f
#endif
#ifndef M1_OBS_SS_SPD_ERR_RAD
#define M1_OBS_SS_SPD_ERR_RAD           0.2617994f
#endif
#ifndef M1_OBS_SS_SPD_DOMEGA_MAX
#define M1_OBS_SS_SPD_DOMEGA_MAX        2500.0f
#endif
#ifndef M1_OBS_SS_SPD_DOMEGA_WIN_S
#define M1_OBS_SS_SPD_DOMEGA_WIN_S      0.10f
#endif
#ifndef M1_OBS_SS_SPD_ENC_MATCH_ENABLE
#define M1_OBS_SS_SPD_ENC_MATCH_ENABLE  0
#endif
#ifndef M1_OBS_SS_SPD_ENC_MATCH_RPM
#define M1_OBS_SS_SPD_ENC_MATCH_RPM     50.0f
#endif
#ifndef M1_OBS_SS_SPD_REQUIRE_EMAG
#define M1_OBS_SS_SPD_REQUIRE_EMAG      0
#endif
#ifndef M1_OBS_SS_SPD_REQUIRE_ERR
#define M1_OBS_SS_SPD_REQUIRE_ERR       0
#endif
#ifndef M1_IF_TO_OBS_ENABLE
#define M1_IF_TO_OBS_ENABLE             0
#endif

#define OBS_SS_PI 3.14159265358979323846f
#define OBS_SS_TWO_PI 6.28318530717958647692f

static obs_ss_state_t s_state;
static float s_alpha;
static float s_arm_s;
static float s_arm_fail_s;
static float s_blend_s;
static float s_rpm_trip_s;
static uint8_t s_hat_valid;
static uint8_t s_spd_on_obs;
static float s_obs_age_s;
static float s_spd_hold_s;
static uint8_t s_domega_inited;
static float s_domega_abs_rpm_s;
static float s_omega_obs_lp;
static float s_omega_obs_lp_anchor;
static float s_domega_win_s;

static float obs_ss_wrap_pi(float x)
{
    while (x > OBS_SS_PI) {
        x -= OBS_SS_TWO_PI;
    }
    while (x < -OBS_SS_PI) {
        x += OBS_SS_TWO_PI;
    }
    return x;
}

static float obs_ss_fabsf(float x)
{
    return (x < 0.0f) ? -x : x;
}

static void obs_ss_spd_reset(void)
{
    s_spd_on_obs = 0u;
    s_obs_age_s = 0.0f;
    s_spd_hold_s = 0.0f;
    s_domega_inited = 0u;
    s_domega_abs_rpm_s = 0.0f;
    s_omega_obs_lp = 0.0f;
    s_omega_obs_lp_anchor = 0.0f;
    s_domega_win_s = 0.0f;
}

void obs_soft_switch_init(void)
{
    obs_soft_switch_reset();
}

void obs_soft_switch_reset(void)
{
    s_state = OBS_SS_ENC;
    s_alpha = 0.0f;
    s_arm_s = 0.0f;
    s_arm_fail_s = 0.0f;
    s_blend_s = 0.0f;
    s_rpm_trip_s = 0.0f;
    s_hat_valid = 0u;
    obs_ss_spd_reset();
}

obs_ss_state_t obs_soft_switch_get_state(void)
{
    return s_state;
}

float obs_soft_switch_get_alpha(void)
{
    return s_alpha;
}

uint8_t obs_soft_switch_speed_on_obs(void)
{
    return (s_state == OBS_SS_OBS) ? 1u : 0u;
}

uint8_t obs_soft_switch_speed_use_obs(void)
{
#if !M1_OBS_SS_SPEED_SWITCH_ENABLE
    return 0u;
#elif M1_ENC_OPTIONAL_ENABLE
    /*
     * 编码器可选：角一旦离开 ENC/ARM，速度环必须吃 ω̂。
     * 1550：仍等 SPD_DEFER 时 fb=enc PLL≈0 → 猛推 → ω̂ 飞车，门控永不过。
     */
    return (s_state == OBS_SS_BLEND || s_state == OBS_SS_OBS) ? 1u : 0u;
#elif M1_OBS_SS_SPD_DEFER_ENABLE
    return s_spd_on_obs;
#else
    return (s_state == OBS_SS_BLEND || s_state == OBS_SS_OBS) ? 1u : 0u;
#endif
}

#if M1_OBS_SS_SPEED_SWITCH_ENABLE && M1_OBS_SS_SPD_DEFER_ENABLE
static void obs_ss_spd_defer_tick(float rpm_enc,
                                  float rpm_obs,
                                  float ref_abs,
                                  float err_abs,
                                  float emag,
                                  float dt)
{
    float rpm_err_max;
    uint8_t spd_gates;

    if (s_state != OBS_SS_OBS) {
        /* 离开 OBS 则清切速武装（回退/重来） */
        if (s_spd_on_obs != 0u || s_obs_age_s > 0.0f) {
            obs_ss_spd_reset();
        }
        return;
    }

    s_obs_age_s += dt;

    /*
     * |dω̂/dt|：ω̂ 轻 LPF 后按长窗 |Δω|/Δt（默认 100 ms）。
     * 单拍/SPEED_TS 求导测的是 2 kHz 台阶噪声（~3e4 rpm/s），会卡死速切（1322）。
     */
    if (s_domega_inited == 0u) {
        s_omega_obs_lp = rpm_obs;
        s_omega_obs_lp_anchor = rpm_obs;
        s_domega_win_s = 0.0f;
        s_domega_abs_rpm_s = 0.0f;
        s_domega_inited = 1u;
    } else {
        s_omega_obs_lp += 0.02f * (rpm_obs - s_omega_obs_lp);
        s_domega_win_s += dt;
        if (s_domega_win_s >= M1_OBS_SS_SPD_DOMEGA_WIN_S) {
            s_domega_abs_rpm_s =
                obs_ss_fabsf(s_omega_obs_lp - s_omega_obs_lp_anchor) /
                s_domega_win_s;
            s_omega_obs_lp_anchor = s_omega_obs_lp;
            s_domega_win_s = 0.0f;
        }
    }

    if (s_spd_on_obs != 0u) {
        return; /* 已切速，保持（本试验不做速回退） */
    }
    if (s_obs_age_s < M1_OBS_SS_SPD_DWELL_S) {
        s_spd_hold_s = 0.0f;
        return;
    }

    rpm_err_max = M1_OBS_SS_SPD_RPM_ERR_FRAC *
                  ((ref_abs > 1.0f) ? ref_abs : M1_OBS_SS_RPM_ENTER);
    /* 切速门限：ω̂↔ω_ref + 长窗 |dω̂/dt|；不看编码器 */
    spd_gates = (obs_ss_fabsf(rpm_obs - ref_abs) <= rpm_err_max) &&
                (s_domega_abs_rpm_s <= M1_OBS_SS_SPD_DOMEGA_MAX);
#if M1_OBS_SS_SPD_REQUIRE_EMAG
    spd_gates = spd_gates && (emag >= M1_OBS_SS_EMAG_MIN);
#else
    (void)emag;
#endif
#if M1_OBS_SS_SPD_REQUIRE_ERR
    spd_gates = spd_gates && (err_abs <= M1_OBS_SS_SPD_ERR_RAD);
#else
    (void)err_abs;
#endif

#if M1_OBS_SS_SPD_ENC_MATCH_ENABLE
    spd_gates = spd_gates &&
                (obs_ss_fabsf(rpm_obs - rpm_enc) <= M1_OBS_SS_SPD_ENC_MATCH_RPM);
#else
    (void)rpm_enc;
#endif

    if (spd_gates != 0u) {
        s_spd_hold_s += dt;
        if (s_spd_hold_s >= M1_OBS_SS_SPD_HOLD_S) {
            s_spd_on_obs = 1u;
        }
    } else {
        s_spd_hold_s = 0.0f;
    }
}
#endif /* SPD_DEFER */

float obs_soft_switch_apply(float theta_enc,
                            float theta_hat,
                            float theta_err,
                            float emag,
                            float omega_enc_rpm,
                            float omega_obs_rpm,
                            float omega_ref,
                            float iq,
                            float dt)
{
    const float rpm_enc = obs_ss_fabsf(omega_enc_rpm);
    const float rpm_obs = obs_ss_fabsf(omega_obs_rpm);
    const float err_abs = obs_ss_fabsf(theta_err);
    const float iq_abs = obs_ss_fabsf(iq);
    const float ref_abs = obs_ss_fabsf(omega_ref);
    /* 进入测速门限放宽到 0.85×ENTER */
    const float rpm_enter_meas = M1_OBS_SS_RPM_ENTER * 0.85f;
    /* OBS/BLEND 掉速看观测速；观测速未就绪时仍用编码器（有感路径） */
    const float rpm_trip =
        ((s_state == OBS_SS_OBS || s_state == OBS_SS_BLEND) && (rpm_obs > 50.0f))
            ? rpm_obs
            : rpm_enc;
    uint8_t gates_ok;
    uint8_t trip;
    uint8_t rpm_low;

    if (dt <= 0.0f) {
        dt = OBS_CTRL_TS_S;
    }

    s_hat_valid = 1u;

    trip = 0u;
#if M1_OBS_SS_FALLBACK_ENABLE
    if (s_state == OBS_SS_BLEND || s_state == OBS_SS_OBS) {
#if M1_OBS_SS_FALLBACK_ON_ERR_ENABLE
        if (err_abs > M1_OBS_SS_ERR_EXIT_RAD) {
            trip = 1u;
        }
#endif
        if (iq_abs > M1_OBS_SS_IQ_ABS_MAX) {
            trip = 1u;
        }
        rpm_low = (ref_abs > M1_OBS_SS_RPM_EXIT && rpm_trip < M1_OBS_SS_RPM_EXIT)
                      ? 1u
                      : 0u;
        if (rpm_low != 0u) {
            s_rpm_trip_s += dt;
            if (s_rpm_trip_s >= M1_OBS_SS_RPM_TRIP_GRACE_S) {
                trip = 1u;
            }
        } else {
            s_rpm_trip_s = 0.0f;
        }
    } else {
        s_rpm_trip_s = 0.0f;
    }

    if (trip != 0u || s_state == OBS_SS_FALLBACK) {
        s_state = OBS_SS_FALLBACK;
        s_alpha = 0.0f;
        s_arm_s = 0.0f;
        s_arm_fail_s = 0.0f;
        s_blend_s = 0.0f;
        s_rpm_trip_s = 0.0f;
        obs_ss_spd_reset();
        return theta_enc;
    }
#else
    (void)iq_abs;
    (void)rpm_trip;
    (void)rpm_low;
    (void)trip;
    s_rpm_trip_s = 0.0f;
    if (s_state == OBS_SS_OBS) {
        s_alpha = 1.0f;
#if M1_OBS_SS_SPEED_SWITCH_ENABLE && M1_OBS_SS_SPD_DEFER_ENABLE
        obs_ss_spd_defer_tick(rpm_enc, rpm_obs, ref_abs, err_abs, emag, dt);
#endif
        return theta_hat;
    }
#endif

#if M1_IF_TO_OBS_ENABLE
    /*
     * I/F→无感：ARM 只看观测速 + emag；期满直接 BLEND（2149：等 θ_if 对齐会空等）。
     */
    gates_ok = (s_hat_valid != 0u) &&
               (ref_abs >= M1_OBS_SS_RPM_ENTER) &&
               (rpm_obs >= rpm_enter_meas) &&
               (emag >= M1_OBS_SS_EMAG_MIN) &&
               (obs_ss_fabsf(rpm_obs - ref_abs) <= (0.20f * M1_OBS_SS_RPM_ENTER));
    (void)rpm_enc;
#else
    /* 有感爬升软切：测速门仍用编码器；角差可连续要求（闭环不漂） */
    gates_ok = (s_hat_valid != 0u) &&
               (ref_abs >= M1_OBS_SS_RPM_ENTER) &&
               (rpm_enc >= rpm_enter_meas) &&
               (emag >= M1_OBS_SS_EMAG_MIN) &&
               (err_abs <= M1_OBS_SS_ERR_ENTER_RAD);
#endif

    switch (s_state) {
    case OBS_SS_ENC:
        s_alpha = 0.0f;
        s_arm_fail_s = 0.0f;
        obs_ss_spd_reset();
        if (gates_ok != 0u) {
            s_state = OBS_SS_ARM;
            s_arm_s = 0.0f;
        }
        return theta_enc;

    case OBS_SS_ARM:
        s_alpha = 0.0f;
        if (gates_ok == 0u) {
            s_arm_fail_s += dt;
            if (s_arm_fail_s >= M1_OBS_SS_ARM_GRACE_S) {
                s_state = OBS_SS_ENC;
                s_arm_s = 0.0f;
                s_arm_fail_s = 0.0f;
            }
            /* grace 内保持 ARM，计时暂停但不清零 */
            return theta_enc;
        }
        s_arm_fail_s = 0.0f;
        s_arm_s += dt;
        if (s_arm_s >= M1_OBS_SS_ARM_S) {
            s_state = OBS_SS_BLEND;
            s_blend_s = 0.0f;
            s_alpha = 0.0f;
        }
        return theta_enc;

    case OBS_SS_BLEND:
        s_blend_s += dt;
        if (M1_OBS_SS_BLEND_S <= 1.0e-6f) {
            s_alpha = 1.0f;
        } else {
            s_alpha = s_blend_s / M1_OBS_SS_BLEND_S;
            if (s_alpha > 1.0f) {
                s_alpha = 1.0f;
            }
        }
        if (s_alpha >= 1.0f) {
            s_state = OBS_SS_OBS;
            s_alpha = 1.0f;
            s_obs_age_s = 0.0f;
            s_spd_hold_s = 0.0f;
            s_spd_on_obs = 0u;
            s_domega_inited = 0u;
#if M1_OBS_SS_SPEED_SWITCH_ENABLE && M1_OBS_SS_SPD_DEFER_ENABLE
            obs_ss_spd_defer_tick(rpm_enc, rpm_obs, ref_abs, err_abs, emag, dt);
#endif
            return theta_hat;
        }
        return theta_enc + s_alpha * obs_ss_wrap_pi(theta_hat - theta_enc);

    case OBS_SS_OBS:
        s_alpha = 1.0f;
#if M1_OBS_SS_SPEED_SWITCH_ENABLE && M1_OBS_SS_SPD_DEFER_ENABLE
        obs_ss_spd_defer_tick(rpm_enc, rpm_obs, ref_abs, err_abs, emag, dt);
#endif
        return theta_hat;

    case OBS_SS_FALLBACK:
    default:
        s_alpha = 0.0f;
        obs_ss_spd_reset();
        return theta_enc;
    }
}

#else /* !M1_OBS_SOFT_SWITCH_ENABLE */

void obs_soft_switch_init(void)
{
}

void obs_soft_switch_reset(void)
{
}

obs_ss_state_t obs_soft_switch_get_state(void)
{
    return OBS_SS_ENC;
}

float obs_soft_switch_get_alpha(void)
{
    return 0.0f;
}

uint8_t obs_soft_switch_speed_on_obs(void)
{
    return 0u;
}

uint8_t obs_soft_switch_speed_use_obs(void)
{
    return 0u;
}

float obs_soft_switch_apply(float theta_enc,
                            float theta_hat,
                            float theta_err,
                            float emag,
                            float omega_enc_rpm,
                            float omega_obs_rpm,
                            float omega_ref,
                            float iq,
                            float dt)
{
    (void)theta_hat;
    (void)theta_err;
    (void)emag;
    (void)omega_enc_rpm;
    (void)omega_obs_rpm;
    (void)omega_ref;
    (void)iq;
    (void)dt;
    return theta_enc;
}

#endif /* M1_OBS_SOFT_SWITCH_ENABLE */
