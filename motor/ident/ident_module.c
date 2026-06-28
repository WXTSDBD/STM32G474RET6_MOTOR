/**
 * @file ident_module.c
 * @brief 堵转辨识激励：Iq 阶跃 + Bode sin（Phase 4 纯波形，无 deadband/dbg）。
 */

#include "ident_module.h"

#include <math.h>
#include <stddef.h>

#include "foc_pi.h"
#include "motor_params_m1.h"
#include "motor_trig.h"

#if M1_IDENT_ENABLE

#if M1_IDENT_IQ_BODE_ENABLE
/** 与 tools/ident/analyze_iq_bode.py FREQS 一致：10 Hz → 800 Hz，×1.15，32 点 */
static const float s_bode_freq_table[] = {
    10.0000f, 11.5000f, 13.2250f, 15.2087f, 17.4901f, 20.1136f, 23.1306f, 26.6002f,
    30.5902f, 35.1788f, 40.4556f, 46.5239f, 53.5025f, 61.5279f, 70.7571f, 81.3706f,
    93.5762f, 107.6126f, 123.7545f, 142.3177f, 163.6654f, 188.2152f, 216.4475f,
    248.9146f, 286.2518f, 329.1895f, 378.5680f, 435.3531f, 500.6561f, 575.7545f,
    662.1177f, 761.4354f,
};

#define IDENT_BODE_FREQ_COUNT  ((uint8_t)(sizeof(s_bode_freq_table) / sizeof(s_bode_freq_table[0])))
#endif

static ident_module_state_t s_state;
static uint32_t s_tick;
static float s_iq_cmd;
static uint8_t s_round;
static uint8_t s_phase_in_round;
#if M1_IDENT_IQ_BODE_ENABLE
static float s_bode_f_hz;
static float s_bode_phase;
static uint32_t s_bode_freq_tick;
static uint8_t s_bode_freq_idx;
#endif

#if M1_IDENT_IQ_STEP_ENABLE
static const float s_step_round[] = {
    M1_IDENT_STEP_I1_A,
    M1_IDENT_STEP_I0_A,
    M1_IDENT_STEP_I2_A,
    M1_IDENT_STEP_I0_A,
    M1_IDENT_STEP_I3_A,
    M1_IDENT_STEP_I0_A,
};

#define IDENT_ROUND_PHASES  (sizeof(s_step_round) / sizeof(s_step_round[0]))

static float ident_phase_dwell_s(uint8_t phase)
{
    (void)phase;
    if (phase < IDENT_ROUND_PHASES && fabsf(s_step_round[phase]) < 0.05f) {
        return M1_IDENT_STEP_ZERO_DWELL_S;
    }
    return M1_IDENT_STEP_DWELL_S;
}
#endif

static uint32_t ident_ticks_from_s(float s)
{
    return (uint32_t)(s / M1_CTRL_TS_S + 0.5f);
}

#if M1_IDENT_IQ_BODE_ENABLE
static void ident_bode_start_freq(void)
{
    s_bode_freq_idx = 0u;
    s_bode_f_hz = s_bode_freq_table[0];
    s_bode_phase = 0.0f;
    s_bode_freq_tick = 0u;
}

static uint32_t ident_bode_ticks_per_freq(void)
{
    if (s_bode_f_hz < 1.0f) {
        return ident_ticks_from_s(0.5f);
    }
    return ident_ticks_from_s(M1_IDENT_BODE_CYCLES_PER_FREQ / s_bode_f_hz);
}

static void ident_bode_begin_round(uint8_t round)
{
    s_round = round;
    ident_bode_start_freq();
}

static void ident_bode_advance_freq(void)
{
    s_bode_freq_idx++;
    if (s_bode_freq_idx >= IDENT_BODE_FREQ_COUNT) {
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

void ident_module_init(motor_context_t *ctx)
{
    if (ctx == NULL) {
        return;
    }

    ctx->mode = M1_CTRL_CURRENT_LOOP;
    ctx->id_ref = 0.0f;
    ctx->iq_ref = M1_IDENT_STEP_I0_A;
    ctx->ud_pi = 0.0f;
    ctx->uq_pi = 0.0f;
    foc_pi_reset(&ctx->pi_id);
    foc_pi_reset(&ctx->pi_iq);

    s_state = IDENT_MOD_HOLD;
    s_tick = 0u;
    s_round = 0u;
    s_phase_in_round = 0u;
    s_iq_cmd = M1_IDENT_STEP_I0_A;
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

void ident_module_tick(motor_context_t *ctx)
{
#if M1_IDENT_IQ_BODE_ENABLE
    const float two_pi = 6.28318530718f;
#endif

    if (ctx == NULL) {
        return;
    }

    ctx->id_ref = 0.0f;
    s_tick++;

    switch (s_state) {
    case IDENT_MOD_HOLD:
        ctx->iq_ref = M1_IDENT_STEP_I0_A;
        s_iq_cmd = ctx->iq_ref;
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
        ctx->iq_ref = s_step_round[s_phase_in_round];
        s_iq_cmd = ctx->iq_ref;
        if (s_tick >= ident_ticks_from_s(ident_phase_dwell_s(s_phase_in_round))) {
            s_tick = 0u;
            s_phase_in_round++;
            if (s_phase_in_round >= IDENT_ROUND_PHASES) {
                s_phase_in_round = 0u;
                s_round++;
                if (s_round >= M1_IDENT_STEP_ROUNDS) {
#if M1_IDENT_IQ_BODE_ENABLE
                    ident_bode_begin_round(0u);
                    s_state = IDENT_MOD_BODE;
#else
                    s_state = IDENT_MOD_DONE;
#endif
                }
            }
        }
        break;
#endif

#if M1_IDENT_IQ_BODE_ENABLE
    case IDENT_MOD_BODE:
        s_bode_phase += two_pi * s_bode_f_hz * M1_CTRL_TS_S;
        if (s_bode_phase >= two_pi) {
            s_bode_phase -= two_pi;
        }
        ctx->iq_ref = M1_IDENT_BODE_I_BIAS_A +
                      M1_IDENT_BODE_I_AMP_A * motor_trig_sin(s_bode_phase);
        s_iq_cmd = ctx->iq_ref;
        s_bode_freq_tick++;
        if (s_bode_freq_tick >= ident_bode_ticks_per_freq()) {
            s_bode_freq_tick = 0u;
            s_bode_phase = 0.0f;
            ident_bode_advance_freq();
        }
        break;
#endif

    case IDENT_MOD_DONE:
    default:
        ctx->iq_ref = M1_IDENT_STEP_I0_A;
        s_iq_cmd = ctx->iq_ref;
        s_round = 0u;
        break;
    }
}

float ident_module_iq_ref_cmd(void)
{
    return s_iq_cmd;
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

float ident_module_bode_freq_hz(void)
{
    return 0.0f;
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
