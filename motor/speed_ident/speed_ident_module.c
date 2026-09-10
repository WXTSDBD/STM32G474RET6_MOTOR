/**
 * @file speed_ident_module.c
 * @brief 速度环辨识：ω_ref 阶跃 + Bode sin（deadband OFF，编码器 θ Park）。
 */

#include "speed_ident_module.h"

#include <math.h>
#include <stddef.h>

#include "foc_pi.h"
#include "motor_params_m1.h"
#include "motor_trig.h"

#if M1_SPEED_IDENT_ENABLE

#if M1_SPEED_IDENT_BODE_ENABLE
static const float s_bode_freq_table[] = {
    0.5000f, 0.6300f, 0.7943f, 1.0000f,
    1.2589f, 1.5849f, 1.9953f, 2.5119f,
    3.1623f, 3.9811f, 5.0119f, 6.3096f,
    7.9433f, 10.0000f, 12.5893f, 15.8489f,
    19.9526f, 25.1189f, 31.6228f, 39.8107f,
    50.0000f,
};

#define SPEED_IDENT_BODE_FREQ_COUNT \
    ((uint8_t)(sizeof(s_bode_freq_table) / sizeof(s_bode_freq_table[0])))
#endif

static speed_ident_module_state_t s_state;
static uint32_t s_tick;
static float s_omega_cmd;
static uint8_t s_round;
static uint8_t s_phase_in_round;
#if M1_SPEED_IDENT_BODE_ENABLE
static float s_bode_f_hz;
static float s_bode_phase;
static uint32_t s_bode_freq_tick;
static uint8_t s_bode_freq_idx;
#endif

#if M1_SPEED_IDENT_STEP_ENABLE
#ifndef M1_SPEED_IDENT_STEP_LADDER_ENABLE
#define M1_SPEED_IDENT_STEP_LADDER_ENABLE  0
#endif
#if M1_SPEED_IDENT_STEP_LADDER_ENABLE
/* 单调阶梯：RPM0→…→RPM6（无回基准）；缺省档位由 motor_params / profile 给出 */
static const float s_step_seq[] = {
    M1_SPEED_IDENT_STEP_RPM0,
    M1_SPEED_IDENT_STEP_RPM1,
    M1_SPEED_IDENT_STEP_RPM2,
    M1_SPEED_IDENT_STEP_RPM3,
    M1_SPEED_IDENT_STEP_RPM4,
    M1_SPEED_IDENT_STEP_RPM5,
    M1_SPEED_IDENT_STEP_RPM6,
};
#else
/* 速度环签收：探档 ↔ 回基准 RPM0 */
static const float s_step_seq[] = {
    M1_SPEED_IDENT_STEP_RPM1,
    M1_SPEED_IDENT_STEP_RPM0,
    M1_SPEED_IDENT_STEP_RPM2,
    M1_SPEED_IDENT_STEP_RPM0,
    M1_SPEED_IDENT_STEP_RPM3,
    M1_SPEED_IDENT_STEP_RPM0,
};
#endif

#define SPEED_IDENT_STEP_PHASES  (sizeof(s_step_seq) / sizeof(s_step_seq[0]))

static float speed_ident_step_baseline(void)
{
    return M1_SPEED_IDENT_STEP_RPM0;
}

static float speed_ident_step_done_rpm(void)
{
#if M1_SPEED_IDENT_STEP_LADDER_ENABLE
    return s_step_seq[SPEED_IDENT_STEP_PHASES - 1u];
#else
    return M1_SPEED_IDENT_STEP_RPM0;
#endif
}

static float speed_ident_phase_dwell_s(uint8_t phase)
{
#if M1_SPEED_IDENT_STEP_LADDER_ENABLE
    (void)phase;
    return M1_SPEED_IDENT_STEP_DWELL_S;
#else
    const float baseline = speed_ident_step_baseline();

    if (phase < SPEED_IDENT_STEP_PHASES &&
        fabsf(s_step_seq[phase] - baseline) < 1.0f) {
        return M1_SPEED_IDENT_STEP_ZERO_DWELL_S;
    }
    return M1_SPEED_IDENT_STEP_DWELL_S;
#endif
}
#endif

static uint32_t speed_ident_ticks_from_s(float s)
{
    return (uint32_t)(s / M1_SPEED_TS_S + 0.5f);
}

#if M1_SPEED_IDENT_BODE_ENABLE
static void speed_ident_bode_start_freq(void)
{
    s_bode_freq_idx = 0u;
    s_bode_f_hz = s_bode_freq_table[0];
    s_bode_phase = 0.0f;
    s_bode_freq_tick = 0u;
}

static uint32_t speed_ident_bode_ticks_per_freq(void)
{
    uint32_t ticks;

    if (s_bode_f_hz < 0.5f) {
        ticks = speed_ident_ticks_from_s(2.0f);
    } else {
        ticks = speed_ident_ticks_from_s(M1_SPEED_IDENT_BODE_CYCLES_PER_FREQ /
                                         s_bode_f_hz);
    }
    return (ticks < 1u) ? 1u : ticks;
}

static void speed_ident_bode_advance_freq(void)
{
    s_bode_freq_idx++;
    if (s_bode_freq_idx >= SPEED_IDENT_BODE_FREQ_COUNT) {
        s_state = SPEED_IDENT_DONE;
    } else {
        s_bode_f_hz = s_bode_freq_table[s_bode_freq_idx];
        s_bode_phase = 0.0f;
        s_bode_freq_tick = 0u;
    }
}
#endif

void speed_ident_module_init(motor_context_t *ctx)
{
    if (ctx == NULL) {
        return;
    }

    ctx->mode = M1_CTRL_CURRENT_LOOP;
    ctx->id_ref = 0.0f;
    ctx->omega_ref = 0.0f;
    ctx->ud_pi = 0.0f;
    ctx->uq_pi = 0.0f;
    foc_pi_reset(&ctx->pi_id);
    foc_pi_reset(&ctx->pi_iq);
    foc_pi_reset(&ctx->pi_speed);

    s_state = SPEED_IDENT_HOLD;
    s_tick = 0u;
    s_round = 0u;
    s_phase_in_round = 0u;
    s_omega_cmd = 0.0f;
#if M1_SPEED_IDENT_BODE_ENABLE
    speed_ident_bode_start_freq();
#endif
}

speed_ident_module_state_t speed_ident_module_get_state(void)
{
    return s_state;
}

uint8_t speed_ident_module_get_round(void)
{
    return s_round;
}

uint8_t speed_ident_module_get_phase_in_round(void)
{
    return s_phase_in_round;
}

void speed_ident_module_tick(motor_context_t *ctx)
{
#if M1_SPEED_IDENT_BODE_ENABLE
    const float two_pi = 6.28318530718f;
#endif

    if (ctx == NULL) {
        return;
    }

    ctx->id_ref = 0.0f;
    s_tick++;

    switch (s_state) {
    case SPEED_IDENT_HOLD:
        {
            const uint32_t settle_ticks =
                speed_ident_ticks_from_s(M1_SPEED_IDENT_PLL_SETTLE_S);
            const uint32_t hold_ticks =
                speed_ident_ticks_from_s(M1_SPEED_IDENT_HOLD_S);

            if (s_tick < settle_ticks) {
                ctx->omega_ref = 0.0f;
                s_omega_cmd = 0.0f;
                break;
            }

            if (hold_ticks > settle_ticks + 1u) {
                const float frac =
                    (float)(s_tick - settle_ticks) /
                    (float)(hold_ticks - settle_ticks);

                ctx->omega_ref = M1_SPEED_IDENT_RPM_START +
                                 (M1_SPEED_IDENT_STEP_RPM0 -
                                  M1_SPEED_IDENT_RPM_START) * frac;
            } else {
                ctx->omega_ref = M1_SPEED_IDENT_STEP_RPM0;
            }
        }
        s_omega_cmd = ctx->omega_ref;
        if (s_tick >= speed_ident_ticks_from_s(M1_SPEED_IDENT_HOLD_S)) {
            s_tick = 0u;
            s_round = 0u;
            s_phase_in_round = 0u;
#if M1_SPEED_IDENT_STEP_ENABLE
            s_state = SPEED_IDENT_STEP;
#elif M1_SPEED_IDENT_BODE_ENABLE
            s_state = SPEED_IDENT_BODE;
#else
            s_state = SPEED_IDENT_DONE;
#endif
        }
        break;

#if M1_SPEED_IDENT_STEP_ENABLE
    case SPEED_IDENT_STEP:
        if (s_phase_in_round >= SPEED_IDENT_STEP_PHASES) {
            s_phase_in_round = 0u;
        }
        ctx->omega_ref = s_step_seq[s_phase_in_round];
        s_omega_cmd = ctx->omega_ref;
        if (s_tick >= speed_ident_ticks_from_s(
                speed_ident_phase_dwell_s(s_phase_in_round))) {
            s_tick = 0u;
            s_phase_in_round++;
            if (s_phase_in_round >= SPEED_IDENT_STEP_PHASES) {
                s_phase_in_round = 0u;
                s_round++;
                if (s_round >= M1_SPEED_IDENT_STEP_ROUNDS) {
#if M1_SPEED_IDENT_BODE_ENABLE
                    s_tick = 0u;
                    speed_ident_bode_start_freq();
                    s_state = SPEED_IDENT_BODE;
#else
                    s_state = SPEED_IDENT_DONE;
                    ctx->omega_ref = speed_ident_step_done_rpm();
                    s_omega_cmd = ctx->omega_ref;
#endif
                }
            }
        }
        break;
#endif

#if M1_SPEED_IDENT_BODE_ENABLE
    case SPEED_IDENT_BODE:
        s_bode_phase += two_pi * s_bode_f_hz * M1_SPEED_TS_S;
        if (s_bode_phase >= two_pi) {
            s_bode_phase -= two_pi;
        }
        ctx->omega_ref = M1_SPEED_IDENT_BODE_RPM_BIAS +
                         M1_SPEED_IDENT_BODE_RPM_AMP *
                             motor_trig_sin(s_bode_phase);
        s_omega_cmd = ctx->omega_ref;
        s_bode_freq_tick++;
        if (s_bode_freq_tick >= speed_ident_bode_ticks_per_freq()) {
            speed_ident_bode_advance_freq();
        }
        if (s_state == SPEED_IDENT_DONE) {
#if M1_SPEED_IDENT_STEP_ENABLE
            ctx->omega_ref = speed_ident_step_done_rpm();
#else
            ctx->omega_ref = M1_SPEED_IDENT_STEP_RPM0;
#endif
            s_omega_cmd = ctx->omega_ref;
        }
        break;
#endif

    case SPEED_IDENT_DONE:
    default:
#if M1_SPEED_IDENT_STEP_ENABLE
        ctx->omega_ref = speed_ident_step_done_rpm();
#else
        ctx->omega_ref = M1_SPEED_IDENT_STEP_RPM0;
#endif
        s_omega_cmd = ctx->omega_ref;
        s_round = 0u;
        break;
    }
}

float speed_ident_module_omega_ref_cmd(void)
{
    return s_omega_cmd;
}

float speed_ident_module_bode_freq_hz(void)
{
#if M1_SPEED_IDENT_BODE_ENABLE
    if (s_state == SPEED_IDENT_BODE) {
        return s_bode_f_hz;
    }
#endif
    return 0.0f;
}

uint8_t speed_ident_module_step_round(void)
{
#if M1_SPEED_IDENT_STEP_ENABLE
    if (s_state == SPEED_IDENT_STEP) {
        return s_round;
    }
#endif
    return 0u;
}

uint8_t speed_ident_module_bode_round(void)
{
#if M1_SPEED_IDENT_BODE_ENABLE
    if (s_state == SPEED_IDENT_BODE) {
        return s_bode_freq_idx;
    }
#endif
    return 0u;
}

uint8_t speed_ident_module_hold_iq_inhibit(void)
{
#if M1_SPEED_IDENT_STEP_ENABLE
    const uint32_t settle_ticks =
        speed_ident_ticks_from_s(M1_SPEED_IDENT_PLL_SETTLE_S);

    return (s_state == SPEED_IDENT_HOLD && s_tick < settle_ticks) ? 1u : 0u;
#else
    return 0u;
#endif
}

#else /* !M1_SPEED_IDENT_ENABLE */

void speed_ident_module_init(motor_context_t *ctx)
{
    (void)ctx;
}

void speed_ident_module_tick(motor_context_t *ctx)
{
    (void)ctx;
}

speed_ident_module_state_t speed_ident_module_get_state(void)
{
    return SPEED_IDENT_DONE;
}

uint8_t speed_ident_module_get_round(void)
{
    return 0u;
}

uint8_t speed_ident_module_get_phase_in_round(void)
{
    return 0u;
}

float speed_ident_module_omega_ref_cmd(void)
{
    return 0.0f;
}

float speed_ident_module_bode_freq_hz(void)
{
    return 0.0f;
}

uint8_t speed_ident_module_step_round(void)
{
    return 0u;
}

uint8_t speed_ident_module_bode_round(void)
{
    return 0u;
}

uint8_t speed_ident_module_hold_iq_inhibit(void)
{
    return 0u;
}

#endif /* M1_SPEED_IDENT_ENABLE */
