/**
 * @file ident_module.c
 * @date 2026-10-06
 * @brief 堵转辨识激励发生器。

 *
 * 节拍限制见 ident_module.h 文件头。
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
 */

#include "ident_module.h"

#include <math.h>
#include <stddef.h>

#include "foc_pi.h"
#include "motor_params_m1.h"
#include "motor_trig.h"

#if M1_IDENT_ENABLE

#if M1_IDENT_IQ_BODE_ENABLE
#include "ident_bode_freq_table.h"

#if (M1_BRINGUP_MODE == M1_BRINGUP_MODE_BODE_OFF_ONLY) || \
    (M1_BRINGUP_MODE == M1_BRINGUP_MODE_BODE_ID_OFF_ONLY)
#if (IDENT_BODE_FREQ_COUNT != 57u)
#error "BODE_OFF/ID: need 57-point table (F1=2500). Run tools/gen_bode_freq_table.py"
#endif
#endif
#endif

static ident_module_state_t s_state;
static uint32_t s_tick;
static float s_iq_cmd;
static float s_id_cmd;
static uint8_t s_round;
static uint8_t s_phase_in_round;
#if M1_IDENT_IQ_BODE_ENABLE
static float s_bode_f_hz;
static float s_bode_phase;
static uint32_t s_bode_freq_tick;
static uint16_t s_bode_freq_idx;
#endif

#if M1_IDENT_IQ_STEP_ENABLE
/** 低段：0→0.3→0→0.5→0→1.0→0 */
static const float s_step_low[] = {
    M1_IDENT_STEP_I1_A,
    M1_IDENT_STEP_I0_A,
    M1_IDENT_STEP_I2_A,
    M1_IDENT_STEP_I0_A,
    M1_IDENT_STEP_I3_A,
    M1_IDENT_STEP_I0_A,
};

#if M1_IDENT_STEP_BANDS > 1u
/** 高段：1.0→1.3→1.0→1.5→1.0→2.0→1.0（round 4..7） */
static const float s_step_high[] = {
    M1_IDENT_STEP_I5_A,
    M1_IDENT_STEP_I_BASE_HI_A,
    M1_IDENT_STEP_I6_A,
    M1_IDENT_STEP_I_BASE_HI_A,
    M1_IDENT_STEP_I7_A,
    M1_IDENT_STEP_I_BASE_HI_A,
};
#endif

#define IDENT_ROUND_PHASES  (sizeof(s_step_low) / sizeof(s_step_low[0]))

static const float *ident_step_table_for_round(uint8_t round)
{
#if M1_IDENT_STEP_BANDS > 1u
    const uint8_t pair = round / (uint8_t)M1_IDENT_STEP_ROUNDS_PER_PROFILE;

    return (pair / 2u >= 1u) ? s_step_high : s_step_low;
#else
    (void)round;
    return s_step_low;
#endif
}

static float ident_step_baseline_for_table(const float *tbl)
{
#if M1_IDENT_STEP_BANDS > 1u
    if (tbl == s_step_high) {
        return M1_IDENT_STEP_I_BASE_HI_A;
    }
#else
    (void)tbl;
#endif
    return M1_IDENT_STEP_I0_A;
}

static float ident_phase_dwell_s(uint8_t phase, const float *tbl)
{
    const float baseline = ident_step_baseline_for_table(tbl);

    if (phase < IDENT_ROUND_PHASES &&
        fabsf(tbl[phase] - baseline) < 0.05f) {
        return M1_IDENT_STEP_ZERO_DWELL_S;
    }
    return M1_IDENT_STEP_DWELL_S;
}
#endif

static uint32_t ident_ticks_from_s(float s)
{
    return (uint32_t)(s / M1_CTRL_TS_S + 0.5f);
}

static void ident_set_dq_ref_idle(motor_context_t *ctx)
{
#if M1_IDENT_BODE_AXIS_ID
    ctx->id_ref = 0.0f;
    ctx->iq_ref = 0.0f;
    s_id_cmd = 0.0f;
    s_iq_cmd = 0.0f;
#else
    ctx->id_ref = 0.0f;
    ctx->iq_ref = M1_IDENT_STEP_I0_A;
    s_id_cmd = 0.0f;
    s_iq_cmd = ctx->iq_ref;
#endif
}

#if M1_IDENT_IQ_BODE_ENABLE
static void ident_bode_start_freq(void)
{
    s_bode_freq_idx = 0u;
    s_bode_f_hz = s_bode_freq_table[0];
    s_bode_phase = 0.0f;
    s_bode_freq_tick = 0u;
}

static float ident_bode_cycles_for_freq(float f_hz)
{
    if (f_hz < 1.0f) {
        return 0.0f;
    }
    if (f_hz >= M1_IDENT_BODE_F_SPLIT_HZ) {
#if M1_IDENT_BODE_USE_T_OBS_HI
        return M1_IDENT_BODE_T_OBS_HI_S * f_hz;
#else
        return M1_IDENT_BODE_CYCLES_HI;
#endif
    }
    return M1_IDENT_BODE_CYCLES_PER_FREQ;
}

static uint32_t ident_bode_ticks_per_freq(void)
{
    uint32_t ticks;

    if (s_bode_f_hz < 1.0f) {
        ticks = ident_ticks_from_s(0.5f);
    } else {
        ticks = ident_ticks_from_s(ident_bode_cycles_for_freq(s_bode_f_hz) / s_bode_f_hz);
    }
    return (ticks < 1u) ? 1u : ticks;
}

static void ident_bode_begin_round(uint8_t round)
{
    s_round = round;
    ident_bode_start_freq();
}

static void ident_bode_i_sin(float *bias_out, float *amp_out)
{
#if M1_IDENT_BODE_BANDS > 1u
#if !M1_IDENT_BODE_LUT_ENABLE
    if (s_round >= 1u) {
#else
    if ((s_round / 2u) >= 1u) {
#endif
        *bias_out = M1_IDENT_BODE_I_BIAS_HI_A;
        *amp_out = M1_IDENT_BODE_I_AMP_HI_A;
        return;
    }
#endif
    *bias_out = M1_IDENT_BODE_I_BIAS_A;
    *amp_out = M1_IDENT_BODE_I_AMP_A;
}

static void ident_bode_apply_sin(motor_context_t *ctx, float bias, float amp, float phase)
{
    const float i_cmd = bias + amp * motor_trig_sin(phase);

#if M1_IDENT_BODE_AXIS_ID
    ctx->id_ref = i_cmd;
    ctx->iq_ref = 0.0f;
    s_id_cmd = i_cmd;
    s_iq_cmd = 0.0f;
#else
    ctx->id_ref = 0.0f;
    ctx->iq_ref = i_cmd;
    s_id_cmd = 0.0f;
    s_iq_cmd = i_cmd;
#endif
}

static void ident_bode_advance_freq(void)
{
    s_bode_freq_idx++;
    if (s_bode_freq_idx >= (uint16_t)IDENT_BODE_FREQ_COUNT) {
        s_round++;
        if (s_round < M1_IDENT_BODE_ROUNDS) {
            ident_bode_begin_round(s_round);
        } else {
            s_state = IDENT_MOD_DONE;
        }
    } else {
        s_bode_f_hz = s_bode_freq_table[s_bode_freq_idx];
    }
}
#endif

/**
 * @brief 从 HOLD 开始。
 */
void ident_module_init(motor_context_t *ctx)
{
    if (ctx == NULL) {
        return;
    }

    ctx->mode = M1_CTRL_CURRENT_LOOP;
    ctx->ud_pi = 0.0f;
    ctx->uq_pi = 0.0f;
    foc_pi_reset(&ctx->pi_id);
    foc_pi_reset(&ctx->pi_iq);

    ident_set_dq_ref_idle(ctx);

    s_state = IDENT_MOD_HOLD;
    s_tick = 0u;
    s_round = 0u;
    s_phase_in_round = 0u;
#if M1_IDENT_IQ_BODE_ENABLE
    ident_bode_start_freq();
#endif
}

ident_module_state_t ident_module_get_state(void)
{
    return s_state;
}

uint8_t ident_module_get_round(void)
{
    return s_round;
}

uint8_t ident_module_get_phase_in_round(void)
{
    return s_phase_in_round;
}

/**
 * @brief 写出本拍 Iq/Id 激励。
 */
void ident_module_tick(motor_context_t *ctx)
{
#if M1_IDENT_IQ_BODE_ENABLE
    const float two_pi = 6.28318530718f;
#endif

    if (ctx == NULL) {
        return;
    }

    s_tick++;

    switch (s_state) {
    case IDENT_MOD_HOLD:
        ident_set_dq_ref_idle(ctx);
        if (s_tick >= ident_ticks_from_s(M1_IDENT_HOLD_S)) {
            s_tick = 0u;
            s_round = 0u;
            s_phase_in_round = 0u;
#if M1_IDENT_IQ_STEP_ENABLE
            s_state = IDENT_MOD_STEP;
#elif M1_IDENT_IQ_BODE_ENABLE
            ident_bode_begin_round(0u);
            s_state = IDENT_MOD_BODE;
#else
            s_state = IDENT_MOD_DONE;
#endif
        }
        break;

#if M1_IDENT_IQ_STEP_ENABLE
    case IDENT_MOD_STEP:
        if (s_phase_in_round >= IDENT_ROUND_PHASES) {
            s_phase_in_round = 0u;
        }
        {
            const float *tbl = ident_step_table_for_round(s_round);

            ctx->iq_ref = tbl[s_phase_in_round];
            s_iq_cmd = ctx->iq_ref;
            ctx->id_ref = 0.0f;
            s_id_cmd = 0.0f;
            if (s_tick >= ident_ticks_from_s(ident_phase_dwell_s(s_phase_in_round, tbl))) {
                s_tick = 0u;
                s_phase_in_round++;
                if (s_phase_in_round >= IDENT_ROUND_PHASES) {
                    s_phase_in_round = 0u;
                    s_round++;
                    if (s_round >= M1_IDENT_STEP_ROUNDS) {
#if M1_IDENT_IQ_BODE_ENABLE
                        s_tick = 0u;
                        ident_bode_begin_round(0u);
                        s_state = IDENT_MOD_BODE;
#else
                        s_state = IDENT_MOD_DONE;
                        ident_set_dq_ref_idle(ctx);
#endif
                    }
                }
            }
        }
        break;
#endif

#if M1_IDENT_IQ_BODE_ENABLE
    case IDENT_MOD_BODE:
        {
            float bode_bias;
            float bode_amp;

            ident_bode_i_sin(&bode_bias, &bode_amp);
            s_bode_phase += two_pi * s_bode_f_hz * M1_CTRL_TS_S;
            if (s_bode_phase >= two_pi) {
                s_bode_phase -= two_pi;
            }
            ident_bode_apply_sin(ctx, bode_bias, bode_amp, s_bode_phase);
        }
        s_bode_freq_tick++;
        if (s_bode_freq_tick >= ident_bode_ticks_per_freq()) {
            s_bode_freq_tick = 0u;
            s_bode_phase = 0.0f;
            ident_bode_advance_freq();
        }
        if (s_state == IDENT_MOD_DONE) {
            ident_set_dq_ref_idle(ctx);
        }
        break;
#endif

    case IDENT_MOD_DONE:
    default:
        ident_set_dq_ref_idle(ctx);
        s_round = 0u;
        break;
    }
}

float ident_module_iq_ref_cmd(void)
{
    return s_iq_cmd;
}

float ident_module_id_ref_cmd(void)
{
    return s_id_cmd;
}

float ident_module_bode_freq_hz(void)
{
#if M1_IDENT_IQ_BODE_ENABLE
    if (s_state == IDENT_MOD_BODE) {
        return s_bode_f_hz;
    }
#endif
    return 0.0f;
}

uint16_t ident_module_bode_freq_idx(void)
{
#if M1_IDENT_IQ_BODE_ENABLE
    if (s_state == IDENT_MOD_BODE) {
        return s_bode_freq_idx;
    }
#endif
    return 0u;
}

uint8_t ident_module_step_round(void)
{
#if M1_IDENT_IQ_STEP_ENABLE
    if (s_state == IDENT_MOD_STEP) {
        return s_round;
    }
#endif
    return 0u;
}

uint8_t ident_module_bode_round(void)
{
#if M1_IDENT_IQ_BODE_ENABLE
    if (s_state == IDENT_MOD_BODE) {
        return s_round;
    }
#endif
    return 0u;
}

uint8_t ident_module_deadband_fixed(void)
{
    return 0u;
}

#else /* !M1_IDENT_ENABLE */

void ident_module_init(motor_context_t *ctx)
{
    (void)ctx;
}

void ident_module_tick(motor_context_t *ctx)
{
    (void)ctx;
}

ident_module_state_t ident_module_get_state(void)
{
    return IDENT_MOD_DONE;
}

uint8_t ident_module_get_round(void)
{
    return 0u;
}

uint8_t ident_module_get_phase_in_round(void)
{
    return 0u;
}

float ident_module_iq_ref_cmd(void)
{
    return 0.0f;
}

float ident_module_id_ref_cmd(void)
{
    return 0.0f;
}

float ident_module_bode_freq_hz(void)
{
    return 0.0f;
}

uint16_t ident_module_bode_freq_idx(void)
{
    return 0u;
}

uint8_t ident_module_step_round(void)
{
    return 0u;
}

uint8_t ident_module_bode_round(void)
{
    return 0u;
}

uint8_t ident_module_deadband_fixed(void)
{
    return 0u;
}

#endif /* M1_IDENT_ENABLE */
