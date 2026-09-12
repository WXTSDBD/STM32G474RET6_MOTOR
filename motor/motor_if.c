/**
 * @file motor_if.c
 * @brief ALIGN → RAMP → HOLD → STOP；θ 斜坡 + 恒 Iq，电流 PI 闭环。
 */

#include "motor_if.h"

#include <stddef.h>

#include "motor_params_m1.h"

#ifndef M1_TWO_PI
#define M1_TWO_PI 6.28318530718f
#endif

#if M1_IF_ENABLE

static m1_if_state_t s_state;
static uint32_t s_tick;
static float s_theta_if;
#if !(M1_IF_TO_OBS_ENABLE || M1_ENC_OPTIONAL_ENABLE)
static float s_theta_enc_prev;
static uint8_t s_theta_enc_prev_valid;
#endif
static float s_rpm_filt;
static uint8_t s_released;
static float s_target_rpm; /* 含符号；DIR_SEQ 零速后再改为反向 */

static uint32_t if_ticks_from_s(float s)
{
    if (s <= 0.0f) {
        return 0u;
    }
    return (uint32_t)(s / M1_CTRL_TS_S + 0.5f);
}

static float if_wrap_0_2pi(float rad)
{
    while (rad >= M1_TWO_PI) {
        rad -= M1_TWO_PI;
    }
    while (rad < 0.0f) {
        rad += M1_TWO_PI;
    }
    return rad;
}

static float if_wrap_pi(float rad)
{
    while (rad > 3.14159265359f) {
        rad -= M1_TWO_PI;
    }
    while (rad < -3.14159265359f) {
        rad += M1_TWO_PI;
    }
    return rad;
}

static float if_clamp_iq(float iq)
{
    if (iq > M1_I_REF_ABS_MAX) {
        return M1_I_REF_ABS_MAX;
    }
    if (iq < -M1_I_REF_ABS_MAX) {
        return -M1_I_REF_ABS_MAX;
    }
    return iq;
}

#if !(M1_IF_TO_OBS_ENABLE || M1_ENC_OPTIONAL_ENABLE)
static void if_update_rpm(float theta_enc_park, uint8_t pole_pairs)
{
    float dtheta;
    float alpha;
    float inst_rpm;
    const float pp = (pole_pairs > 0u) ? (float)pole_pairs : 1.0f;

    if (!s_theta_enc_prev_valid) {
        s_theta_enc_prev = theta_enc_park;
        s_theta_enc_prev_valid = 1u;
        s_rpm_filt = 0.0f;
        return;
    }

    dtheta = if_wrap_pi(theta_enc_park - s_theta_enc_prev);
    s_theta_enc_prev = theta_enc_park;
    inst_rpm = (dtheta / M1_CTRL_TS_S / pp) * 60.0f / M1_TWO_PI;
    if (inst_rpm < 0.0f) {
        inst_rpm = -inst_rpm;
    }

    alpha = M1_CTRL_TS_S / 0.050f;
    if (alpha > 1.0f) {
        alpha = 1.0f;
    }
    s_rpm_filt += alpha * (inst_rpm - s_rpm_filt);
}
#endif

static float if_omega_el_tgt(uint8_t pole_pairs)
{
    const float pp = (pole_pairs > 0u) ? (float)pole_pairs : 1.0f;

    return s_target_rpm * pp * (M1_TWO_PI / 60.0f);
}

/**
 * @brief 按指令转速取 Iq：中速危险带可抬高，缩短斜坡后补同步裕度。
 */
static float if_iq_for_cmd_rpm(float omega_cmd_rpm)
{
    float iq = M1_IF_IQ_A;

#if M1_IF_IQ_MID_BOOST_ENABLE
    {
        float abs_rpm = omega_cmd_rpm;
        if (abs_rpm < 0.0f) {
            abs_rpm = -abs_rpm;
        }
        /* 正/反向共用 |ω| 中速带抬 Iq */
        if ((abs_rpm >= M1_IF_IQ_MID_RPM_LO) &&
            (abs_rpm <= M1_IF_IQ_MID_RPM_HI)) {
            iq = M1_IF_IQ_MID_A;
        }
    }
#else
    (void)omega_cmd_rpm;
#endif
    return if_clamp_iq(iq);
}

static void if_goto_run(void)
{
    s_state = M1_IF_ALIGN;
    s_tick = 0u;
    s_theta_if = 0.0f;
#if !(M1_IF_TO_OBS_ENABLE || M1_ENC_OPTIONAL_ENABLE)
    s_theta_enc_prev_valid = 0u;
#endif
    s_rpm_filt = 0.0f;
    s_released = 0u;
}

void motor_if_init(motor_context_t *ctx)
{
    (void)ctx;
    s_target_rpm = M1_IF_TARGET_RPM;
    if_goto_run();
}

void motor_if_arm(motor_context_t *ctx)
{
    (void)ctx;
    if_goto_run();
}

void motor_if_release(void)
{
    s_released = 1u;
    s_state = M1_IF_STOP;
    s_tick = 0u;
}

void motor_if_set_target_rpm(float rpm)
{
    s_target_rpm = rpm;
}

float motor_if_get_target_rpm(void)
{
    return s_target_rpm;
}

uint8_t motor_if_is_driving(void)
{
    if (s_released != 0u) {
        return 0u;
    }
    if (s_state == M1_IF_STOP || s_state == M1_IF_FAULT) {
        return 0u;
    }
    return 1u;
}

m1_if_state_t motor_if_get_state(void)
{
    return s_state;
}

motor_if_step_t motor_if_tick(motor_context_t *ctx, float theta_enc_park)
{
    motor_if_step_t out;
    const uint32_t align_ticks = if_ticks_from_s(M1_IF_ALIGN_S);
    const uint32_t ramp_ticks = if_ticks_from_s(M1_IF_RAMP_S);
    const uint32_t hold_ticks = if_ticks_from_s(M1_IF_HOLD_S);
    const float id_cmd = if_clamp_iq(M1_IF_ID_A);
    float iq_cmd;
    float omega_el = 0.0f;
    float t;

    out.state = s_state;
    out.theta_park = theta_enc_park;
    out.iq_ref = 0.0f;
    out.id_ref = 0.0f;
    out.omega_cmd_rpm = 0.0f;
    out.omega_meas_rpm = s_rpm_filt;
    out.theta_err_rad = 0.0f;

    if (s_released != 0u) {
        out.state = M1_IF_STOP;
        return out;
    }

    if (s_state == M1_IF_RAMP || s_state == M1_IF_HOLD) {
#if M1_IF_TO_OBS_ENABLE || M1_ENC_OPTIONAL_ENABLE
        /* 无感路径：测速用开环指令，不微分编码器（可拔） */
        (void)theta_enc_park;
#else
        if_update_rpm(theta_enc_park, ctx->pole_pairs);
        out.omega_meas_rpm = s_rpm_filt;
#endif
    }

    switch (s_state) {
    case M1_IF_ALIGN:
        s_theta_if = 0.0f;
        out.theta_park = 0.0f;
        iq_cmd = if_iq_for_cmd_rpm(0.0f);
        out.iq_ref = iq_cmd;
        out.id_ref = id_cmd;
        out.omega_cmd_rpm = 0.0f;
        out.theta_err_rad = if_wrap_pi(0.0f - theta_enc_park);
        s_tick++;
        if (align_ticks == 0u || s_tick >= align_ticks) {
            s_state = M1_IF_RAMP;
            s_tick = 0u;
            /*
             * ALIGN 把转子拉到驱动电角 0；无感/可选编码器须从 0 起斜坡。
             * 旧逻辑用 θ_enc 续相：拔编码器或零偏未校准时会立刻跳角。
             */
#if M1_IF_TO_OBS_ENABLE || M1_ENC_OPTIONAL_ENABLE
            s_theta_if = 0.0f;
#else
            s_theta_if = if_wrap_0_2pi(theta_enc_park);
            s_theta_enc_prev_valid = 0u;
#endif
        }
        break;

    case M1_IF_RAMP:
        if (ramp_ticks == 0u) {
            t = 1.0f;
        } else {
            t = (float)s_tick / (float)ramp_ticks;
            if (t > 1.0f) {
                t = 1.0f;
            }
        }
        omega_el = if_omega_el_tgt(ctx->pole_pairs) * t;
        s_theta_if = if_wrap_0_2pi(s_theta_if + omega_el * M1_CTRL_TS_S);
        out.theta_park = s_theta_if;
        out.omega_cmd_rpm = s_target_rpm * t;
        iq_cmd = if_iq_for_cmd_rpm(out.omega_cmd_rpm);
        out.iq_ref = iq_cmd;
        out.id_ref = id_cmd;
#if M1_IF_TO_OBS_ENABLE || M1_ENC_OPTIONAL_ENABLE
        s_rpm_filt = out.omega_cmd_rpm;
        out.omega_meas_rpm = s_rpm_filt;
#endif
        out.theta_err_rad = if_wrap_pi(s_theta_if - theta_enc_park);
        s_tick++;
        if (ramp_ticks == 0u || s_tick >= ramp_ticks) {
            s_state = M1_IF_HOLD;
            s_tick = 0u;
        }
        break;

    case M1_IF_HOLD:
        omega_el = if_omega_el_tgt(ctx->pole_pairs);
        s_theta_if = if_wrap_0_2pi(s_theta_if + omega_el * M1_CTRL_TS_S);
        out.theta_park = s_theta_if;
        out.omega_cmd_rpm = s_target_rpm;
        iq_cmd = if_iq_for_cmd_rpm(out.omega_cmd_rpm);
        out.iq_ref = iq_cmd;
        out.id_ref = id_cmd;
#if M1_IF_TO_OBS_ENABLE || M1_ENC_OPTIONAL_ENABLE
        s_rpm_filt = out.omega_cmd_rpm;
        out.omega_meas_rpm = s_rpm_filt;
#endif
        out.theta_err_rad = if_wrap_pi(s_theta_if - theta_enc_park);
        s_tick++;
        if (hold_ticks > 0u && s_tick >= hold_ticks) {
            s_state = M1_IF_STOP;
            s_tick = 0u;
        }
        break;

    case M1_IF_STOP:
    case M1_IF_FAULT:
    default:
        out.theta_park = theta_enc_park;
        out.iq_ref = 0.0f;
        out.id_ref = 0.0f;
        out.omega_cmd_rpm = 0.0f;
        out.theta_err_rad = 0.0f;
        break;
    }

    out.state = s_state;
    return out;
}

#else /* !M1_IF_ENABLE */

void motor_if_init(motor_context_t *ctx)
{
    (void)ctx;
}

void motor_if_arm(motor_context_t *ctx)
{
    (void)ctx;
}

void motor_if_release(void)
{
}

uint8_t motor_if_is_driving(void)
{
    return 0u;
}

void motor_if_set_target_rpm(float rpm)
{
    (void)rpm;
}

float motor_if_get_target_rpm(void)
{
    return 0.0f;
}

m1_if_state_t motor_if_get_state(void)
{
    return M1_IF_STOP;
}

motor_if_step_t motor_if_tick(motor_context_t *ctx, float theta_enc_park)
{
    motor_if_step_t out;

    (void)ctx;
    out.state = M1_IF_STOP;
    out.theta_park = theta_enc_park;
    out.iq_ref = 0.0f;
    out.id_ref = 0.0f;
    out.omega_cmd_rpm = 0.0f;
    out.omega_meas_rpm = 0.0f;
    out.theta_err_rad = 0.0f;
    return out;
}

#endif /* M1_IF_ENABLE */
