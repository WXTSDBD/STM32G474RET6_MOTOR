/**
 * @file motor_current.c
 * @brief M1 JEOC：θ → 采样 → Park → Id/Iq PI → SVPWM → kick → telem。
 */

#include "motor_current.h"

#include <stddef.h>

#include "encoder.h"
#include "FOC_CAL.h"
#include "app_uart_dma_debug.h"
#include "deadband.h"
#include "foc_pi.h"
#include "main.h"
#include "motor_phase_binding.h"
#include "motor_params_m1.h"
#include "motor_trig.h"
#include "trans.h"

static motor_context_t s_m1_ctx;

#if M1_OPEN_SEQ_ENABLE
static uint32_t s_open_seq_tick;
static uint8_t s_open_seq_armed = 1u;
#endif

#define M1_ACDC_WINDOW_TICKS  10000u

static void motor_current_update_acdc(float id, float iq)
{
    static uint32_t tick;
    static float id_min;
    static float id_max;
    static float iq_min;
    static float iq_max;
    static float id_sum;
    static float iq_sum;
    float id_mean;
    float iq_mean;
    float id_pp;
    float iq_pp;

    if (tick == 0u) {
        id_min = id;
        id_max = id;
        iq_min = iq;
        iq_max = iq;
        id_sum = 0.0f;
        iq_sum = 0.0f;
    } else {
        if (id < id_min) {
            id_min = id;
        }
        if (id > id_max) {
            id_max = id;
        }
        if (iq < iq_min) {
            iq_min = iq;
        }
        if (iq > iq_max) {
            iq_max = iq;
        }
    }

    id_sum += id;
    iq_sum += iq;
    tick++;

    if (tick < M1_ACDC_WINDOW_TICKS) {
        return;
    }

    id_mean = id_sum / (float)M1_ACDC_WINDOW_TICKS;
    iq_mean = iq_sum / (float)M1_ACDC_WINDOW_TICKS;
    id_pp = id_max - id_min;
    iq_pp = iq_max - iq_min;

    if (id_mean >= 0.0f) {
        dbg.id_acdc = (id_mean > 0.05f) ? (id_pp / (2.0f * id_mean)) : id_pp;
    } else {
        dbg.id_acdc = (id_mean < -0.05f) ? (id_pp / (-2.0f * id_mean)) : id_pp;
    }

    if (iq_mean >= 0.0f) {
        dbg.iq_acdc = (iq_mean > 0.05f) ? (iq_pp / (2.0f * iq_mean)) : iq_pp;
    } else {
        dbg.iq_acdc = (iq_mean < -0.05f) ? (iq_pp / (-2.0f * iq_mean)) : iq_pp;
    }

    tick = 0u;
}

static float motor_current_clamp_ref(float ref)
{
    if (ref > M1_I_REF_ABS_MAX) {
        return M1_I_REF_ABS_MAX;
    }
    if (ref < -M1_I_REF_ABS_MAX) {
        return -M1_I_REF_ABS_MAX;
    }
    return ref;
}

#if M1_CURRENT_RECON_ENABLE
/** 与 setPhaseVoltage 同拍 Uq（含 open_seq 静止段） */
static float motor_current_uq_for_sector(const motor_context_t *ctx)
{
#if M1_OPEN_SEQ_ENABLE
    if (s_open_seq_armed) {
        const uint32_t idle_ticks =
            (uint32_t)(M1_OPEN_IDLE_S / M1_CTRL_TS_S + 0.5f);

        if (s_open_seq_tick < idle_ticks) {
            return 0.0f;
        }
        return M1_OPEN_UQ_RUN_V;
    }
#endif
    if (ctx->mode == M1_CTRL_CURRENT_LOOP) {
        return ctx->uq_pi;
    }
    return ctx->uq_open;
}

static float motor_current_ud_for_sector(const motor_context_t *ctx)
{
    if (ctx->mode == M1_CTRL_CURRENT_LOOP) {
        return ctx->ud_pi;
    }
    return 0.0f;
}

/** 扇区 1～2：C 相 duty 最小，KCL 重建 ic（3～5 实测已可信，勿动 ia/ib） */
static void motor_current_reconstruct_abc(const motor_context_t *ctx,
                                           float theta_svpwm,
                                           float *ia, float *ib, float *ic)
{
    float Uq;
    float Ud;
    int sec;
    float a;
    float b;
    float c;

    if (ctx->mode == M1_CTRL_CURRENT_LOOP) {
        return;
    }

    Uq = motor_current_uq_for_sector(ctx);
    Ud = motor_current_ud_for_sector(ctx);
    sec = svpwm_sector_from_uq_ud(Uq, Ud, theta_svpwm);
    if (sec != 1 && sec != 2) {
        return;
    }

    a = (*ia >= 0.0f) ? *ia : -*ia;
    b = (*ib >= 0.0f) ? *ib : -*ib;
    c = (*ic >= 0.0f) ? *ic : -*ic;
    if (a < M1_CURRENT_RECON_MIN_A && b < M1_CURRENT_RECON_MIN_A &&
        c < M1_CURRENT_RECON_MIN_A) {
        return;
    }

    *ic = -(*ia + *ib);
}
#endif

static void motor_current_pi_init(motor_context_t *ctx)
{
    foc_pi_init(&ctx->pi_id, M1_PI_KP_ID, M1_PI_KI,
                M1_PI_V_LIMIT_MIN, M1_PI_V_LIMIT_V,
                M1_PI_INT_LIMIT_MIN, M1_PI_INT_LIMIT_V);
    foc_pi_init(&ctx->pi_iq, M1_PI_KP_IQ, M1_PI_KI,
                M1_PI_V_LIMIT_MIN, M1_PI_V_LIMIT_V,
                M1_PI_INT_LIMIT_MIN, M1_PI_INT_LIMIT_V);
}

void motor_current_init(bsp_axis_t *axis)
{
    if (axis == NULL) {
        return;
    }

    s_m1_ctx.pole_pairs = (uint8_t)M1_POLE_PAIRS;
#if M1_OPEN_SEQ_ENABLE
    s_m1_ctx.uq_open = 0.0f;
    s_open_seq_tick = 0u;
    s_open_seq_armed = 1u;
    dbg.open_seq_phase = 0u;
#else
    s_m1_ctx.uq_open = M1_OPEN_UQ_RUN_V;
    dbg.open_seq_phase = 1u;
#endif
    s_m1_ctx.mode = M1_CTRL_MODE_DEFAULT;
    s_m1_ctx.id_ref = 0.0f;
    s_m1_ctx.iq_ref = 0.0f;
    s_m1_ctx.ud_pi = 0.0f;
    s_m1_ctx.uq_pi = 0.0f;
    motor_current_pi_init(&s_m1_ctx);
    deadband_init();
    axis->motor_ctx = &s_m1_ctx;
}

motor_context_t *motor_current_ctx(const bsp_axis_t *axis)
{
    if (axis == NULL) {
        return NULL;
    }
    return (motor_context_t *)axis->motor_ctx;
}

void motor_current_set_mode(bsp_axis_t *axis, m1_ctrl_mode_t mode)
{
    motor_context_t *ctx = motor_current_ctx(axis);

    if (ctx == NULL) {
        return;
    }

    if (mode == M1_CTRL_CURRENT_LOOP && ctx->mode != M1_CTRL_CURRENT_LOOP) {
        foc_pi_reset(&ctx->pi_id);
        foc_pi_reset(&ctx->pi_iq);
        ctx->ud_pi = 0.0f;
        ctx->uq_pi = 0.0f;
    }

    ctx->mode = mode;
}

void motor_current_tick(bsp_axis_t *axis, ADC_HandleTypeDef *hadc)
{
    motor_context_t *ctx;
    volatile uint32_t isr_t0 = *(volatile uint32_t *)&DWT->CYCCNT;
    uint16_t enc_raw;
    float i_alpha;
    float i_beta;
    float ia;
    float ib;
    float ic;
    float theta;
    float theta_park;
    float sin_el;
    float cos_el;
    float id;
    float iq;
    float id_ref;
    float iq_ref;
    float ud_out;
    float uq_out;

    if (axis == NULL || axis->enc == NULL || axis->pwm_tim == NULL) {
        return;
    }

    ctx = motor_current_ctx(axis);
    if (ctx == NULL) {
        return;
    }

    if (axis->adc.cal_active) {
        return;
    }

    enc_raw = encoder_get_raw(axis->enc);
    as5047_spi1.raw = (int)enc_raw;
    theta = encoder_get_theta_el(axis->enc, enc_raw, ctx->pole_pairs, as5047_spi1.add);
#if M1_THETA_NEGATE
    theta_park = -theta;
#else
    theta_park = theta;
#endif
    as5047_spi1.get = theta;
    dbg.foc_theta_el = theta_park;

    adc_sample_jeoc_foc(&axis->adc, hadc);
    {
        float i_phys[3];

        adc_sample_get_abc(&axis->adc, &i_phys[0], &i_phys[1], &i_phys[2]);
        motor_phase_binding_map_abc(i_phys, &ia, &ib, &ic);
#if M1_CURRENT_RECON_ENABLE
        motor_current_reconstruct_abc(ctx, theta, &ia, &ib, &ic);
#endif
    }
    dbg.foc_ia = ia;
    dbg.foc_ib = ib;
    dbg.foc_ic = ic;
    Clarke_Transform(ia, ib, ic, &i_alpha, &i_beta);

    motor_trig_sincos(theta_park, &cos_el, &sin_el);
    Park_Transform_sc(i_alpha, i_beta, sin_el, cos_el, &id, &iq);
    dbg.foc_id = id;
    dbg.foc_iq = iq;
    motor_current_update_acdc(id, iq);

    id_ref = motor_current_clamp_ref(ctx->id_ref);
    iq_ref = motor_current_clamp_ref(ctx->iq_ref);

    ctx->ud_pi = foc_pi_step(&ctx->pi_id, id_ref, id);
    ctx->uq_pi = foc_pi_step(&ctx->pi_iq, iq_ref, iq);
    dbg.foc_ud_pi = ctx->ud_pi;
    dbg.foc_uq_pi = ctx->uq_pi;

#if M1_OPEN_SEQ_ENABLE
    if (s_open_seq_armed) {
        const uint32_t idle_ticks =
            (uint32_t)(M1_OPEN_IDLE_S / M1_CTRL_TS_S + 0.5f);

        if (s_open_seq_tick < idle_ticks) {
            ctx->uq_open = 0.0f;
            dbg.open_seq_phase = 0u;
        } else {
            ctx->uq_open = M1_OPEN_UQ_RUN_V;
            dbg.open_seq_phase = 1u;
            s_open_seq_armed = 0u;
        }
        s_open_seq_tick++;
    }
#endif

    switch (ctx->mode) {
    case M1_CTRL_CURRENT_LOOP:
        ud_out = ctx->ud_pi;
        uq_out = ctx->uq_pi;
        break;
    case M1_CTRL_OBSERVE_ONLY:
        ud_out = 0.0f;
        uq_out = ctx->uq_open;
        break;
    case M1_CTRL_OPEN_LOOP:
    default:
        ud_out = 0.0f;
        uq_out = ctx->uq_open;
        break;
    }

    dbg.foc_uq_out = uq_out;

    setPhaseVoltage_abc(axis->pwm_tim, uq_out, ud_out, theta, ia, ib, ic);
    encoder_kick(axis->enc);
    telem_bringup_tick();

    {
        volatile uint32_t isr_t1 = *(volatile uint32_t *)&DWT->CYCCNT;

        g_telem_dbg.isr_t0 = isr_t0;
        g_telem_dbg.isr_t1 = isr_t1;
        g_telem_dbg.cyccnt_end = isr_t1;
        g_telem_dbg.isr_delta = isr_t1 - isr_t0;
    }
}
