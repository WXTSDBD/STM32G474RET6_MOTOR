/**
 * @file motor_startup.c
 * @date 2026-10-06
 * @brief Uq 开环拖动，再无扰切到编码器电流闭环。
 *
 * 节拍限制见 motor_startup.h 文件头。
 *
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
 */

#include "motor_startup.h"

#include <stddef.h>

#include "motor_params_m1.h"

#ifndef M1_TWO_PI
#define M1_TWO_PI 6.28318530718f
#endif

#if M1_STARTUP_ENABLE

/** 当前启动状态。 */
static m1_startup_state_t s_state;
/** 状态内拍计数。 */
static uint32_t s_tick;
/** 开环电角，单位 rad。 */
static float s_theta_open;
/** 上一拍编码器电角，用来估转速。 */
static float s_theta_enc_prev;
/** 1=已经有上一拍编码器角。 */
static uint8_t s_theta_enc_prev_valid;
/** 对照机械转速低通，单位 rpm。 */
static float s_rpm_filt;
/** 进入闭环后的拍计数。 */
static uint32_t s_closed_tick;
/** 切环那一拍的 Iq，单位 A。 */
static float s_iq_at_switch;
/** 1=本拍应对 PI 做无扰。 */
static uint8_t s_bumpless_arm;

static uint32_t startup_ticks_from_s(float s)
{
    if (s <= 0.0f) {
        return 0u;
    }
    return (uint32_t)(s / M1_CTRL_TS_S + 0.5f);
}

static float startup_clamp_ref(float ref)
{
    if (ref > M1_I_REF_ABS_MAX) {
        return M1_I_REF_ABS_MAX;
    }
    if (ref < -M1_I_REF_ABS_MAX) {
        return -M1_I_REF_ABS_MAX;
    }
    return ref;
}

static float startup_wrap_0_2pi(float rad)
{
    while (rad >= M1_TWO_PI) {
        rad -= M1_TWO_PI;
    }
    while (rad < 0.0f) {
        rad += M1_TWO_PI;
    }
    return rad;
}

static float startup_wrap_pi(float rad)
{
    while (rad > 3.14159265359f) {
        rad -= M1_TWO_PI;
    }
    while (rad < -3.14159265359f) {
        rad += M1_TWO_PI;
    }
    return rad;
}

static void startup_update_rpm(float theta_enc_park, uint8_t pole_pairs)
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

    dtheta = startup_wrap_pi(theta_enc_park - s_theta_enc_prev);
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

static void startup_goto_run(void)
{
    if (M1_STARTUP_ALIGN_S > 0.0f) {
        s_state = M1_STARTUP_ALIGN;
    } else {
        s_state = M1_STARTUP_DRAG;
    }
    s_tick = 0u;
    s_theta_open = 0.0f;
    s_theta_enc_prev_valid = 0u;
    s_rpm_filt = 0.0f;
    s_closed_tick = 0u;
    s_iq_at_switch = 0.0f;
    s_bumpless_arm = 0u;
}

static float startup_iq_ref_ramped(motor_context_t *ctx)
{
    const float target = startup_clamp_ref(ctx->iq_ref);
    const uint32_t ramp_ticks = startup_ticks_from_s(M1_STARTUP_IQ_RAMP_S);

    if (ramp_ticks == 0u) {
        return target;
    }

    if (s_closed_tick >= ramp_ticks) {
        return target;
    }

    {
        const float t = (float)s_closed_tick / (float)ramp_ticks;

        return s_iq_at_switch + (target - s_iq_at_switch) * t;
    }
}

/**
 * @brief 清启动状态。
 */
void motor_startup_init(motor_context_t *ctx)
{
    (void)ctx;
    startup_goto_run();
}

void motor_startup_arm(motor_context_t *ctx)
{
    (void)ctx;
    startup_goto_run();
}

/**
 * @brief 读当前启动状态。
 */
m1_startup_state_t motor_startup_get_state(void)
{
    return s_state;
}

motor_startup_step_t motor_startup_tick(motor_context_t *ctx, float theta_enc_park)
{
    motor_startup_step_t out;
    const uint32_t align_ticks = startup_ticks_from_s(M1_STARTUP_ALIGN_S);
    const uint32_t drag_ticks = startup_ticks_from_s(M1_STARTUP_DRAG_S);
    const uint32_t blend_ticks = startup_ticks_from_s(M1_STARTUP_THETA_BLEND_S);
    float t;
    float omega_el_cmd;

    out.pi_reset = 0u;
    out.pi_bumpless = 0u;
    out.uq_prev = 0.0f;
    out.ud_prev = 0.0f;
    out.use_fixed_uq = 0u;
    out.uq_out = 0.0f;
    out.omega_mech_rpm = s_rpm_filt;
    out.iq_ref = 0.0f;

    if (s_state == M1_STARTUP_DRAG || s_state == M1_STARTUP_CLOSED) {
        startup_update_rpm(theta_enc_park, ctx->pole_pairs);
        out.omega_mech_rpm = s_rpm_filt;
    }

    switch (s_state) {
    case M1_STARTUP_ALIGN:
        s_theta_open = 0.0f;
        out.theta_park = 0.0f;
        out.use_fixed_uq = 1u;
        out.uq_out = 0.0f;
        out.iq_ref = 0.0f;
        s_tick++;
        if (align_ticks == 0u || s_tick >= align_ticks) {
            s_state = M1_STARTUP_DRAG;
            s_tick = 0u;
            s_theta_open = startup_wrap_0_2pi(theta_enc_park);
            s_theta_enc_prev_valid = 0u;
        }
        break;

    case M1_STARTUP_DRAG:
        if (s_tick == 0u) {
            s_theta_open = startup_wrap_0_2pi(theta_enc_park);
        }
        if (drag_ticks > 0u) {
            t = (float)s_tick / (float)drag_ticks;
            if (t > 1.0f) {
                t = 1.0f;
            }
        } else {
            t = 1.0f;
        }
        omega_el_cmd = M1_STARTUP_OMEGA_EL_RADS * t;
        s_theta_open = startup_wrap_0_2pi(s_theta_open + omega_el_cmd * M1_CTRL_TS_S);

        if (blend_ticks > 0u && drag_ticks > 0u &&
            s_tick + blend_ticks >= drag_ticks) {
            const float theta_err = startup_wrap_pi(theta_enc_park - s_theta_open);

            s_theta_open = startup_wrap_0_2pi(s_theta_open +
                theta_err * M1_STARTUP_THETA_BLEND_ALPHA);
        }

        out.theta_park = s_theta_open;
        out.use_fixed_uq = 1u;
        out.uq_out = M1_STARTUP_DRAG_UQ_V;
        out.iq_ref = 0.0f;

        s_tick++;
        if (drag_ticks == 0u || s_tick >= drag_ticks) {
            s_state = M1_STARTUP_CLOSED;
            s_closed_tick = 0u;
            s_bumpless_arm = 1u;
            out.theta_park = theta_enc_park;
            out.use_fixed_uq = 0u;
            out.uq_prev = M1_STARTUP_DRAG_UQ_V;
            out.ud_prev = 0.0f;
        }
        break;

    case M1_STARTUP_CLOSED:
        out.theta_park = theta_enc_park;
        out.iq_ref = startup_iq_ref_ramped(ctx);
        s_closed_tick++;
        break;

    case M1_STARTUP_FAULT:
    default:
        out.theta_park = theta_enc_park;
        out.iq_ref = 0.0f;
        break;
    }

    out.state = s_state;
    return out;
}

/**
 * @brief Park 后：Iq 斜坡和切环无扰标志，必须同拍。
 * @param iq_meas 测量 Iq，单位 A。
 */
void motor_startup_finish_tick(motor_context_t *ctx, float iq_meas,
                               motor_startup_step_t *step)
{
    if (ctx == NULL || step == NULL) {
        return;
    }

    if (s_bumpless_arm) {
        s_iq_at_switch = iq_meas;
        s_bumpless_arm = 0u;
        step->pi_bumpless = 1u;
        step->uq_prev = M1_STARTUP_DRAG_UQ_V;
        step->ud_prev = 0.0f;
        step->iq_ref = s_iq_at_switch;
        s_closed_tick = 1u;
    } else if (step->state == M1_STARTUP_CLOSED) {
        step->iq_ref = startup_iq_ref_ramped(ctx);
    }
}

#else /* M1_STARTUP_ENABLE */

void motor_startup_init(motor_context_t *ctx)
{
    (void)ctx;
}

void motor_startup_arm(motor_context_t *ctx)
{
    (void)ctx;
}

m1_startup_state_t motor_startup_get_state(void)
{
    return M1_STARTUP_CLOSED;
}

motor_startup_step_t motor_startup_tick(motor_context_t *ctx, float theta_enc_park)
{
    motor_startup_step_t out;
    float ref = ctx->iq_ref;

    if (ref > M1_I_REF_ABS_MAX) {
        ref = M1_I_REF_ABS_MAX;
    } else if (ref < -M1_I_REF_ABS_MAX) {
        ref = -M1_I_REF_ABS_MAX;
    }

    out.state = M1_STARTUP_CLOSED;
    out.theta_park = theta_enc_park;
    out.iq_ref = ref;
    out.uq_out = 0.0f;
    out.use_fixed_uq = 0u;
    out.omega_mech_rpm = 0.0f;
    out.pi_reset = 0u;
    out.pi_bumpless = 0u;
    out.uq_prev = 0.0f;
    out.ud_prev = 0.0f;
    return out;
}

void motor_startup_finish_tick(motor_context_t *ctx, float iq_meas,
                               motor_startup_step_t *step)
{
    /*
     * STARTUP 关闭时：startup_tick 在 HFI 写 iq_ref 之前取样，foc_loop 在
     * outer=DISABLED 时吃的是 startup.iq_ref。此处跟到本拍 ctx，避免踢段
     * 用上一拍的 0（C4g 2142：Iq 遥测像在跟，力矩环却可能慢半拍/错拍）。
     */
    (void)iq_meas;
    if (ctx != NULL && step != NULL) {
        step->iq_ref = ctx->iq_ref;
    }
}

#endif /* M1_STARTUP_ENABLE */
