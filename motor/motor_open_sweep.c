/**
 * @file motor_open_sweep.c
 * @date 2026-10-06
 * @brief 开环电压扫参：V012 与固定 Uq。
 *
 * 节拍限制见 motor_open_sweep.h 文件头。
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
 */

#include "motor_open_sweep.h"

#include "dbg_monitor.h"
#include "motor_params_m1.h"

typedef enum {
    SWEEP_KIND_NONE = 0,
    SWEEP_KIND_V012,
    SWEEP_KIND_AB_UNUSED,
    SWEEP_KIND_FIXED,
} sweep_kind_t;

static sweep_kind_t s_kind;
static uint32_t s_tick;
static uint8_t s_armed;
static uint32_t s_fixed_duration_ticks;
static float s_fixed_uq_v;

static void motor_open_sweep_clear_open_voltages(motor_context_t *ctx)
{
    if (ctx != NULL) {
        ctx->uq_open = 0.0f;
        ctx->ud_open = 0.0f;
    }
}

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
}

void motor_open_sweep_arm_v012(motor_context_t *ctx)
{
    if (ctx == NULL) {
        return;
    }

    s_kind = SWEEP_KIND_V012;
    s_tick = 0u;
    s_armed = 1u;
    ctx->uq_open = M1_OPEN_UQ_SWEEP_V0;
    ctx->ud_open = 0.0f;
    DBG.open_seq_phase = 0u;
}

void motor_open_sweep_begin_fixed(motor_context_t *ctx, float uq_v, float duration_s)
{
    if (ctx == NULL) {
        return;
    }

    s_kind = SWEEP_KIND_FIXED;
    s_tick = 0u;
    s_armed = 1u;
    s_fixed_uq_v = uq_v;
    if (duration_s <= 0.0f) {
        s_fixed_duration_ticks = 0u;
    } else {
        s_fixed_duration_ticks =
            (uint32_t)(duration_s / M1_CTRL_TS_S + 0.5f);
    }
    ctx->uq_open = uq_v;
    ctx->ud_open = 0.0f;
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
        ctx->ud_open = 0.0f;
        DBG.open_seq_phase = 80u;
        if (t >= s_fixed_duration_ticks) {
            s_armed = 0u;
            s_kind = SWEEP_KIND_NONE;
            motor_open_sweep_clear_open_voltages(ctx);
            DBG.open_seq_phase = 81u;
        }
        return;
    }

    if (s_kind == SWEEP_KIND_V012) {
        const uint32_t step_ticks =
            (uint32_t)(M1_OPEN_UQ_SWEEP_STEP_S / M1_CTRL_TS_S + 0.5f);
        const uint32_t t = s_tick++;

        if (t < step_ticks) {
            ctx->uq_open = M1_OPEN_UQ_SWEEP_V0;
            DBG.open_seq_phase = 0u;
        } else if (t < (2u * step_ticks)) {
            ctx->uq_open = M1_OPEN_UQ_SWEEP_V1;
            DBG.open_seq_phase = 1u;
        } else {
            ctx->uq_open = M1_OPEN_UQ_SWEEP_V2;
            DBG.open_seq_phase = 2u;
            if (t >= (3u * step_ticks)) {
                s_armed = 0u;
                s_kind = SWEEP_KIND_NONE;
            }
        }
        ctx->ud_open = 0.0f;
    }
}
