/**
 * @file motor_open_sweep.c
 * @brief 开环 Uq/Ud 扫参 / 固定 Uq 定时（从 motor_current 外提，R3）。
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
#if M1_OPEN_UQ_PRE_ID_CAL_ENABLE || M1_OPEN_UD_PRE_ID_CAL_ENABLE
    SWEEP_KIND_PRE_ID_LADDER,
#endif
} sweep_kind_t;

#if M1_OPEN_UQ_PRE_ID_CAL_ENABLE || M1_OPEN_UD_PRE_ID_CAL_ENABLE
typedef enum {
    PRE_ID_SUB_LADDER = 0,
    PRE_ID_SUB_ALIGN,
} pre_id_sub_t;

typedef enum {
    PRE_ID_AXIS_UQ = 0,
    PRE_ID_AXIS_UD,
} pre_id_axis_t;

static const float s_pre_id_v_ladder[M1_OPEN_PRE_ID_LADDER_N] = {
    0.0f, 0.2f, 0.5f, 1.0f, 2.0f, M1_OPEN_PRE_ID_LADDER_V4,
};
#define PRE_ID_V_LADDER_LEN  M1_OPEN_PRE_ID_LADDER_N
#endif

static sweep_kind_t s_kind;
static uint32_t s_tick;
static uint8_t s_armed;
static uint32_t s_fixed_duration_ticks;
static float s_fixed_uq_v;
#if M1_OPEN_UQ_PRE_ID_CAL_ENABLE || M1_OPEN_UD_PRE_ID_CAL_ENABLE
static uint8_t s_ladder_step;
static pre_id_sub_t s_pre_id_sub;
static pre_id_axis_t s_pre_id_axis;
#if M1_OPEN_PRE_ID_LADDER_AB_ENABLE
static uint8_t s_ladder_ab_round;
#endif
#endif
#if M1_OPEN_UQ_DEADBAND_AB_SWEEP
static uint32_t s_ab_last_phase;
#endif

static void motor_open_sweep_clear_open_voltages(motor_context_t *ctx)
{
    if (ctx != NULL) {
        ctx->uq_open = 0.0f;
        ctx->ud_open = 0.0f;
    }
}

#if M1_OPEN_UQ_PRE_ID_CAL_ENABLE || M1_OPEN_UD_PRE_ID_CAL_ENABLE
static uint8_t motor_open_sweep_ladder_phase_base(void)
{
#if M1_OPEN_PRE_ID_LADDER_AB_ENABLE
    if (s_ladder_ab_round != 0u) {
        return M1_OPEN_PRE_ID_LADDER_PHASE_ON_BASE;
    }
#endif
    return M1_OPEN_PRE_ID_LADDER_PHASE_BASE;
}

static uint8_t motor_open_sweep_ladder_done_phase(void)
{
#if M1_OPEN_PRE_ID_LADDER_AB_ENABLE
    if (s_ladder_ab_round != 0u) {
        return M1_OPEN_PRE_ID_LADDER_DONE_ON_PHASE;
    }
#endif
    return M1_OPEN_PRE_ID_LADDER_DONE_PHASE;
}

static void motor_open_sweep_apply_pre_id_ladder_voltage(motor_context_t *ctx, float v)
{
    if (ctx == NULL) {
        return;
    }

    if (s_pre_id_axis == PRE_ID_AXIS_UD) {
        ctx->ud_open = v;
        ctx->uq_open = 0.0f;
    } else {
        ctx->uq_open = v;
        ctx->ud_open = 0.0f;
    }
}

static void motor_open_sweep_begin_pre_id_ladder_on_round(motor_context_t *ctx)
{
    s_ladder_ab_round = 1u;
    s_ladder_step = 0u;
    s_tick = 0u;
    s_pre_id_sub = PRE_ID_SUB_LADDER;
#if M1_OPEN_PRE_ID_LADDER_LUT_RUNTIME
    deadband_service_apply_profile(DEADBAND_PROFILE_LUT_RUNTIME);
#else
    deadband_service_apply_profile(DEADBAND_PROFILE_FIXED);
#endif
    motor_open_sweep_apply_pre_id_ladder_voltage(ctx, s_pre_id_v_ladder[0]);
    dbg.open_seq_phase = M1_OPEN_PRE_ID_LADDER_PHASE_ON_BASE;
}

static void motor_open_sweep_begin_pre_id_ladder_common(motor_context_t *ctx)
{
    if (ctx == NULL) {
        return;
    }

    s_ladder_step = 0u;
    s_tick = 0u;
    s_armed = 1u;
    s_kind = SWEEP_KIND_PRE_ID_LADDER;
#if M1_OPEN_PRE_ID_LADDER_AB_ENABLE
    s_ladder_ab_round = 0u;
#endif
#if M1_OPEN_PRE_ID_LADDER_ALIGN_ENABLE
    s_pre_id_sub = PRE_ID_SUB_ALIGN;
    motor_open_sweep_clear_open_voltages(ctx);
    dbg.open_seq_phase = M1_OPEN_PRE_ID_LADDER_ALIGN_PHASE;
#else
    s_pre_id_sub = PRE_ID_SUB_LADDER;
    motor_open_sweep_apply_pre_id_ladder_voltage(ctx, s_pre_id_v_ladder[0]);
    dbg.open_seq_phase = motor_open_sweep_ladder_phase_base();
#endif
}
#endif /* pre-id ladder */

void motor_open_sweep_init(motor_context_t *ctx)
{
    if (ctx != NULL) {
        ctx->uq_open = M1_OPEN_UQ_SWEEP_V0;
        ctx->ud_open = 0.0f;
    }

    s_kind = SWEEP_KIND_NONE;
    s_tick = 0u;
    s_armed = 0u;
    s_fixed_duration_ticks = 0u;
    s_fixed_uq_v = 0.0f;
#if M1_OPEN_UQ_PRE_ID_CAL_ENABLE || M1_OPEN_UD_PRE_ID_CAL_ENABLE
    s_ladder_step = 0u;
    s_pre_id_sub = PRE_ID_SUB_LADDER;
    s_pre_id_axis = PRE_ID_AXIS_UQ;
#if M1_OPEN_PRE_ID_LADDER_AB_ENABLE
    s_ladder_ab_round = 0u;
#endif
#endif
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
    ctx->ud_open = 0.0f;
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
    ctx->ud_open = 0.0f;
    s_tick = 0u;
    s_armed = 1u;
    s_kind = SWEEP_KIND_FIXED;
    s_fixed_duration_ticks = (uint32_t)(duration_s / M1_CTRL_TS_S + 0.5f);
    if (s_fixed_duration_ticks == 0u) {
        s_fixed_duration_ticks = 1u;
    }
}

#if M1_OPEN_UQ_PRE_ID_CAL_ENABLE
void motor_open_sweep_begin_pre_id_uq_ladder(motor_context_t *ctx)
{
    s_pre_id_axis = PRE_ID_AXIS_UQ;
    motor_open_sweep_begin_pre_id_ladder_common(ctx);
}
#endif

#if M1_OPEN_UD_PRE_ID_CAL_ENABLE
void motor_open_sweep_begin_pre_id_ud_ladder(motor_context_t *ctx)
{
    s_pre_id_axis = PRE_ID_AXIS_UD;
    motor_open_sweep_begin_pre_id_ladder_common(ctx);
}
#endif

#if M1_OPEN_UQ_PRE_ID_CAL_ENABLE || M1_OPEN_UD_PRE_ID_CAL_ENABLE
uint8_t motor_open_sweep_use_fix_theta(void)
{
#if M1_OPEN_PRE_ID_LADDER_FIX_THETA_ENABLE
    if (!s_armed || s_kind != SWEEP_KIND_PRE_ID_LADDER) {
        return 0u;
    }
    return 1u;
#else
    return 0u;
#endif
}

uint8_t motor_open_sweep_in_pre_id_align(void)
{
#if M1_OPEN_PRE_ID_LADDER_ALIGN_ENABLE
    return (s_armed != 0u && s_kind == SWEEP_KIND_PRE_ID_LADDER &&
            s_pre_id_sub == PRE_ID_SUB_ALIGN) ?
           1u :
           0u;
#else
    return 0u;
#endif
}

uint8_t motor_open_sweep_in_pre_id_ud_ladder(void)
{
#if M1_OPEN_UD_PRE_ID_CAL_ENABLE
    return (s_armed != 0u && s_kind == SWEEP_KIND_PRE_ID_LADDER &&
            s_pre_id_axis == PRE_ID_AXIS_UD &&
            s_pre_id_sub == PRE_ID_SUB_LADDER) ?
           1u :
           0u;
#else
    return 0u;
#endif
}

#endif

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

#if M1_OPEN_UQ_PRE_ID_CAL_ENABLE || M1_OPEN_UD_PRE_ID_CAL_ENABLE
    if (s_kind == SWEEP_KIND_PRE_ID_LADDER) {
#if M1_OPEN_PRE_ID_LADDER_ALIGN_ENABLE
        if (s_pre_id_sub == PRE_ID_SUB_ALIGN) {
            const uint32_t align_ticks =
                (uint32_t)(M1_OPEN_PRE_ID_LADDER_ALIGN_S / M1_CTRL_TS_S + 0.5f);
            const uint32_t t = s_tick++;

            motor_open_sweep_clear_open_voltages(ctx);
            dbg.open_seq_phase = M1_OPEN_PRE_ID_LADDER_ALIGN_PHASE;
            if (align_ticks == 0u || t + 1u >= align_ticks) {
                s_pre_id_sub = PRE_ID_SUB_LADDER;
                s_tick = 0u;
                s_ladder_step = 0u;
                motor_open_sweep_apply_pre_id_ladder_voltage(ctx, s_pre_id_v_ladder[0]);
                dbg.open_seq_phase = motor_open_sweep_ladder_phase_base();
            }
            return;
        }
#endif
        {
            const uint32_t dwell_ticks =
                (uint32_t)(M1_OPEN_PRE_ID_LADDER_DWELL_S / M1_CTRL_TS_S + 0.5f);
            const uint32_t t = s_tick++;

            motor_open_sweep_apply_pre_id_ladder_voltage(ctx,
                                                         s_pre_id_v_ladder[s_ladder_step]);
            dbg.open_seq_phase =
                (uint8_t)(motor_open_sweep_ladder_phase_base() + s_ladder_step);
            if (dwell_ticks == 0u || t + 1u >= dwell_ticks) {
                s_tick = 0u;
                s_ladder_step++;
                if (s_ladder_step >= PRE_ID_V_LADDER_LEN) {
#if M1_OPEN_PRE_ID_LADDER_AB_ENABLE
                    if (s_ladder_ab_round == 0u) {
                        motor_open_sweep_begin_pre_id_ladder_on_round(ctx);
                        return;
                    }
#endif
                    s_armed = 0u;
                    s_kind = SWEEP_KIND_NONE;
                    s_pre_id_sub = PRE_ID_SUB_LADDER;
                    motor_open_sweep_clear_open_voltages(ctx);
                    dbg.open_seq_phase = motor_open_sweep_ladder_done_phase();
                }
            }
        }
        return;
    }
#endif

    if (s_kind == SWEEP_KIND_FIXED) {
        const uint32_t t = s_tick++;

        ctx->uq_open = s_fixed_uq_v;
        ctx->ud_open = 0.0f;
        dbg.open_seq_phase = 80u;
        if (t >= s_fixed_duration_ticks) {
            s_armed = 0u;
            s_kind = SWEEP_KIND_NONE;
            motor_open_sweep_clear_open_voltages(ctx);
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
        ctx->ud_open = 0.0f;

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
        ctx->ud_open = 0.0f;
    }
}
