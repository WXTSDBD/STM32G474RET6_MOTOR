/**
 * @file obs_soft_switch.c
 * @brief 软切状态机：门限武装 → α 融合 → 观测角；可选超差/过流/掉速回编码器。
 */

#include "obs_soft_switch.h"

#include "motor_params_m1.h"

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

#define OBS_SS_PI 3.14159265358979323846f
#define OBS_SS_TWO_PI 6.28318530717958647692f

static obs_ss_state_t s_state;
static float s_alpha;
static float s_arm_s;
static float s_arm_fail_s;
static float s_blend_s;
static float s_rpm_trip_s;
static uint8_t s_hat_valid;

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
    /* 进入：以 ω_ref 为主；测速门限放宽到 0.85×，避免 PLL 毛刺打断 ARM */
    const float rpm_enter_meas = M1_OBS_SS_RPM_ENTER * 0.85f;
    /* OBS/BLEND 掉速看观测速，避免编码器毛刺误踢（1351 包）；
     * 观测速未就绪时仍用编码器，避免 0rpm 误踢 */
    const float rpm_trip =
        ((s_state == OBS_SS_OBS || s_state == OBS_SS_BLEND) && (rpm_obs > 50.0f))
            ? rpm_obs
            : rpm_enc;
    uint8_t gates_ok;
    uint8_t trip;
    uint8_t rpm_low;

    if (dt <= 0.0f) {
        dt = M1_CTRL_TS_S;
    }

    s_hat_valid = 1u;

    trip = 0u;
    if (s_state == OBS_SS_BLEND || s_state == OBS_SS_OBS) {
#if M1_OBS_SS_FALLBACK_ON_ERR_ENABLE
        if (err_abs > M1_OBS_SS_ERR_EXIT_RAD) {
            trip = 1u;
        }
#endif
        if (iq_abs > M1_OBS_SS_IQ_ABS_MAX) {
            trip = 1u;
        }
        /* 指令仍在 EXIT 以上、观测/测速掉到 EXIT 以下 → 掉速回退
         * （旧版写死 ref>900，低速探底时无法触发） */
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
        return theta_enc;
    }

    gates_ok = (s_hat_valid != 0u) &&
               (ref_abs >= M1_OBS_SS_RPM_ENTER) &&
               (rpm_enc >= rpm_enter_meas) &&
               (emag >= M1_OBS_SS_EMAG_MIN) &&
               (err_abs <= M1_OBS_SS_ERR_ENTER_RAD);

    switch (s_state) {
    case OBS_SS_ENC:
        s_alpha = 0.0f;
        s_arm_fail_s = 0.0f;
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
            return theta_hat;
        }
        return theta_enc + s_alpha * obs_ss_wrap_pi(theta_hat - theta_enc);

    case OBS_SS_OBS:
        s_alpha = 1.0f;
        return theta_hat;

    case OBS_SS_FALLBACK:
    default:
        s_alpha = 0.0f;
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
