/**
 * @file motor_open_sweep.c
 * @brief 开环 Uq 扫参 / 固定 Uq 定时（从 motor_current 外提，R3）。
 */

#include "motor_open_sweep.h"

#include "dbg_monitor.h"
#include "deadband_module.h"
#include "motor_params_m1.h"

typedef enum {
    SWEEP_KIND_NONE = 0,
    SWEEP_KIND_V012,
    SWEEP_KIND_AB,
    SWEEP_KIND_FIXED,
} sweep_kind_t;

static sweep_kind_t s_kind;
static uint32_t s_tick;
static uint8_t s_armed;
static uint32_t s_fixed_duration_ticks;
static float s_fixed_uq_v;
#if M1_OPEN_UQ_DEADBAND_AB_SWEEP
static uint32_t s_ab_last_phase;
#endif

void motor_open_sweep_init(motor_context_t *ctx)
{
    if (ctx != NULL) {
        ctx->uq_open = M1_OPEN_UQ_SWEEP_V0;
    }

    s_kind = SWEEP_KIND_NONE;
    s_tick = 0u;
    s_armed = 0u;
    s_fixed_duration_ticks = 0u;
    s_fixed_uq_v = 0.0f;
#if M1_OPEN_UQ_DEADBAND_AB_SWEEP
    s_ab_last_phase = 0xffffffffu;
#endif
}

void motor_open_sweep_arm_v012(motor_context_t *ctx)
{
    if (ctx == NULL) {
        return;
    }

    ctx->uq_open = M1_OPEN_UQ_SWEEP_V0;
    s_tick = 0u;
    s_armed = 1u;
#if M1_OPEN_UQ_DEADBAND_AB_SWEEP
    s_kind = SWEEP_KIND_AB;
    s_ab_last_phase = 0xffffffffu;
#else
    s_kind = SWEEP_KIND_V012;
#endif
}

void motor_open_sweep_begin_fixed(motor_context_t *ctx, float uq_v, float duration_s)
{
    if (ctx == NULL) {
        return;
    }

    s_fixed_uq_v = uq_v;
    ctx->uq_open = uq_v;
    s_tick = 0u;
    s_armed = 1u;
    s_kind = SWEEP_KIND_FIXED;
    s_fixed_duration_ticks = (uint32_t)(duration_s / M1_CTRL_TS_S + 0.5f);
    if (s_fixed_duration_ticks == 0u) {
        s_fixed_duration_ticks = 1u;
    }
}

uint8_t motor_open_sweep_active(void)
{
    return (s_armed != 0u && s_kind != SWEEP_KIND_NONE) ? 1u : 0u;
}

uint8_t motor_open_sweep_done(void)
{
    return (s_armed == 0u && s_kind == SWEEP_KIND_NONE) ? 1u : 0u;
}

void motor_open_sweep_tick(motor_context_t *ctx)
{
    if (ctx == NULL || !s_armed || s_kind == SWEEP_KIND_NONE) {
        return;
    }

    if (ctx->mode != M1_CTRL_OBSERVE_ONLY && ctx->mode != M1_CTRL_OPEN_LOOP) {
        return;
    }

    if (s_kind == SWEEP_KIND_FIXED) {
        const uint32_t t = s_tick++;

        ctx->uq_open = s_fixed_uq_v;
        dbg.open_seq_phase = 80u;
        if (t >= s_fixed_duration_ticks) {
            s_armed = 0u;
            s_kind = SWEEP_KIND_NONE;
            ctx->uq_open = 0.0f;
            dbg.open_seq_phase = 81u;
        }
        return;
    }

#if M1_OPEN_UQ_DEADBAND_AB_SWEEP
    if (s_kind == SWEEP_KIND_AB) {
        const uint32_t half_ticks =
            (uint32_t)(M1_OPEN_UQ_HALF_STEP_S / M1_CTRL_TS_S + 0.5f);
        const uint32_t t = s_tick++;
        uint32_t phase = (half_ticks > 0u) ? (t / half_ticks) : 0u;

        if (phase >= 6u) {
            phase = 5u;
            s_armed = 0u;
            s_kind = SWEEP_KIND_NONE;
        }

        dbg.open_seq_phase = (uint8_t)phase;

        switch (phase / 2u) {
        case 0u:
            ctx->uq_open = M1_OPEN_UQ_SWEEP_V0;
            break;
        case 1u:
            ctx->uq_open = M1_OPEN_UQ_SWEEP_V1;
            break;
        default:
            ctx->uq_open = M1_OPEN_UQ_SWEEP_V2;
            break;
        }

        if (phase != s_ab_last_phase) {
            s_ab_last_phase = phase;
            if ((phase & 1u) != 0u) {
#if M1_DEADBAND_ENABLE
                deadband_service_apply_profile(DEADBAND_PROFILE_FIXED);
#else
                deadband_service_apply_profile(DEADBAND_PROFILE_OFF);
#endif
            } else {
                deadband_service_apply_profile(DEADBAND_PROFILE_OFF);
            }
        }
        return;
    }
#endif

    if (s_kind == SWEEP_KIND_V012) {
        const uint32_t step_ticks =
            (uint32_t)(M1_OPEN_UQ_SWEEP_STEP_S / M1_CTRL_TS_S + 0.5f);
        const uint32_t t = s_tick++;

        if (t < step_ticks) {
            ctx->uq_open = M1_OPEN_UQ_SWEEP_V0;
            dbg.open_seq_phase = 0u;
        } else if (t < (2u * step_ticks)) {
            ctx->uq_open = M1_OPEN_UQ_SWEEP_V1;
            dbg.open_seq_phase = 1u;
        } else {
            ctx->uq_open = M1_OPEN_UQ_SWEEP_V2;
            dbg.open_seq_phase = 2u;
            if (t >= (3u * step_ticks)) {
                s_armed = 0u;
                s_kind = SWEEP_KIND_NONE;
            }
        }
    }
}
