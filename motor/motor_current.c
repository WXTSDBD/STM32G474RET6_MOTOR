/**
 * @file motor_current.c
 * @brief M1 JEOC：θ → 采样 → Park → foc_loop → SVPWM → kick → telem。
 */

#include "motor_current.h"

#include <stddef.h>

#include "encoder.h"
#include "app_uart_dma_debug.h"
#include "deadband_flow.h"
#include "deadband_id_cal.h"
#include "deadband_module.h"
#include "dbg_monitor.h"
#include "foc_svpwm.h"
#include "motor_foc_loop.h"
#include "motor_open_sweep.h"
#include "motor_phase_binding.h"
#include "motor_params_m1.h"
#include "motor_startup.h"
#include "motor_trig.h"

static motor_context_t s_m1_ctx;

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

#if M1_CURRENT_RECON_ENABLE
static float motor_current_uq_for_sector(const motor_context_t *ctx)
{
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

void motor_current_init(bsp_axis_t *axis)
{
    if (axis == NULL) {
        return;
    }

    s_m1_ctx.pole_pairs = (uint8_t)M1_POLE_PAIRS;
    motor_open_sweep_init(&s_m1_ctx);
#if M1_IDENT_ENABLE || M1_ID_LOCK_CAL_SWEEP
    s_m1_ctx.mode = M1_CTRL_CURRENT_LOOP;
    s_m1_ctx.id_ref = 0.0f;
    deadband_flow_boot(&s_m1_ctx);
#else
    dbg.open_seq_phase = 0u;
    s_m1_ctx.mode = M1_CTRL_MODE_DEFAULT;
    s_m1_ctx.id_ref = 0.0f;
#if M1_STARTUP_ENABLE
    s_m1_ctx.iq_ref = M1_STARTUP_IQ_REF_A;
#else
    s_m1_ctx.iq_ref = M1_IQ_REF_A;
#endif
    if (s_m1_ctx.mode == M1_CTRL_OBSERVE_ONLY ||
        s_m1_ctx.mode == M1_CTRL_OPEN_LOOP) {
        motor_open_sweep_arm_v012(&s_m1_ctx);
    }
#endif
    s_m1_ctx.ud_pi = 0.0f;
    s_m1_ctx.uq_pi = 0.0f;
    motor_foc_loop_pi_init(&s_m1_ctx);
#if M1_STARTUP_ENABLE
    motor_startup_init(&s_m1_ctx);
#endif
    {
        deadband_service_boot_t db_boot;

        deadband_service_boot(&db_boot);
        dbg.deadband_nvm_loaded = db_boot.nvm_loaded;
        dbg.deadband_mode = (uint8_t)deadband_service_get_mode();
    }
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
        motor_foc_loop_pi_reset(ctx);
#if M1_STARTUP_ENABLE
        motor_startup_arm(ctx);
#endif
    }

    ctx->mode = mode;
}

void motor_current_set_idq_ref(bsp_axis_t *axis, float id_ref, float iq_ref)
{
    motor_context_t *ctx = motor_current_ctx(axis);

    if (ctx == NULL) {
        return;
    }

    ctx->id_ref = id_ref;
    ctx->iq_ref = iq_ref;
}

void motor_startup_arm_axis(bsp_axis_t *axis)
{
#if M1_STARTUP_ENABLE
    motor_context_t *ctx = motor_current_ctx(axis);

    if (ctx == NULL) {
        return;
    }

    motor_foc_loop_pi_reset(ctx);
    motor_startup_arm(ctx);
#else
    (void)axis;
#endif
}

void motor_current_tick(bsp_axis_t *axis)
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
    float theta_enc_park;
    float theta_park;
    float sin_el;
    float cos_el;
    float id;
    float iq;
    float ud_out;
    float uq_out;
    motor_startup_step_t startup;

    if (axis == NULL || axis->enc == NULL || axis->pwm == NULL) {
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
    theta = encoder_get_theta_el(axis->enc, enc_raw, ctx->pole_pairs,
                                 encoder_get_theta_el_offset(axis->enc));
#if M1_THETA_NEGATE
    theta_enc_park = -theta;
#else
    theta_enc_park = theta;
#endif

    {
        float i_phys[3];

        adc_sample_get_abc(&axis->adc, &i_phys[0], &i_phys[1], &i_phys[2]);
        motor_phase_binding_map_abc(i_phys, &ia, &ib, &ic);
#if M1_CURRENT_RECON_ENABLE
        motor_current_reconstruct_abc(ctx, theta_enc_park, &ia, &ib, &ic);
#endif
    }
    dbg.foc_ia = ia;
    dbg.foc_ib = ib;
    dbg.foc_ic = ic;
    Clarke_Transform(ia, ib, ic, &i_alpha, &i_beta);

#if M1_IDENT_ENABLE || M1_ID_LOCK_CAL_SWEEP
    deadband_flow_tick(ctx);
#endif

    if (ctx->mode == M1_CTRL_CURRENT_LOOP) {
#if M1_IDENT_ENABLE || M1_ID_LOCK_CAL_SWEEP
        motor_foc_loop_on_flow_tick(ctx);
#endif
        startup = motor_startup_tick(ctx, theta_enc_park);
#if M1_IDENT_ENABLE
#if M1_IDENT_FIX_THETA_ENABLE
        theta_park = M1_IDENT_THETA_EL_RAD;
#else
        theta_park = theta_enc_park;
#endif
#elif M1_ID_LOCK_CAL_SWEEP && M1_ID_CAL_FIX_THETA_ENABLE
        if (deadband_id_cal_use_fix_theta()) {
#if M1_ID_CAL_DUAL_ANGLE_ENABLE
            theta_park = deadband_id_cal_target_theta();
#else
            theta_park = M1_ID_CAL_THETA_EL_RAD;
#endif
        } else {
            theta_park = theta_enc_park;
        }
#else
        theta_park = startup.theta_park;
#endif
        dbg.startup_state = (uint8_t)startup.state;
        dbg.startup_omega_mech_rpm = startup.omega_mech_rpm;
    } else {
        theta_park = theta_enc_park;
        dbg.startup_state = (uint8_t)M1_STARTUP_CLOSED;
        dbg.startup_omega_mech_rpm = 0.0f;
        startup.iq_ref = 0.0f;
        startup.state = M1_STARTUP_CLOSED;
        startup.use_fixed_uq = 0u;
        startup.uq_out = 0.0f;
        startup.pi_bumpless = 0u;
        startup.uq_prev = 0.0f;
        startup.ud_prev = 0.0f;
    }
    dbg.foc_theta_el = theta_park;

    motor_trig_sincos(theta_park, &cos_el, &sin_el);
    Park_Transform_sc(i_alpha, i_beta, sin_el, cos_el, &id, &iq);
    dbg.foc_id = id;
    dbg.foc_iq = iq;
    motor_foc_loop_dbg_id_ref(ctx);
    motor_current_update_acdc(id, iq);

    if (ctx->mode == M1_CTRL_CURRENT_LOOP) {
        motor_startup_finish_tick(ctx, iq, &startup);
        motor_foc_loop_tick(ctx, id, iq, &startup, theta_enc_park);
    }

    if (ctx->mode == M1_CTRL_OBSERVE_ONLY || ctx->mode == M1_CTRL_OPEN_LOOP) {
        motor_open_sweep_tick(ctx);
    }

    switch (ctx->mode) {
    case M1_CTRL_CURRENT_LOOP:
#if M1_ID_CAL_ALIGN_ENABLE && M1_ID_CAL_FIX_THETA_ENABLE
        if (deadband_id_cal_use_align_ud()) {
#if M1_ID_CAL_DUAL_ANGLE_ENABLE
            ud_out = deadband_id_cal_align_ud_v();
#else
            ud_out = M1_ID_CAL_ALIGN_UD_V;
#endif
            uq_out = 0.0f;
        } else if (startup.use_fixed_uq) {
#else
        if (startup.use_fixed_uq) {
#endif
            ud_out = 0.0f;
            uq_out = startup.uq_out;
        } else {
            ud_out = ctx->ud_pi + deadband_service_ud_inject(id);
            uq_out = ctx->uq_pi;
        }
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

    foc_svpwm_apply_abc(axis, uq_out, ud_out, theta_park, ia, ib, ic, id, iq);
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
