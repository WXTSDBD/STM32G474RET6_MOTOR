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
#include "ld_lq_ident.h"
#include "motor_foc_loop.h"
#include "motor_open_sweep.h"
#include "motor_outer_loop.h"
#include "motor_phase_binding.h"
#include "motor_params_m1.h"
#if M1_PLL_ENABLE
#include "motor_pll.h"
#endif
#include "motor_startup.h"
#include "motor_trig.h"
#if M1_EMF_VEQ_ENABLE
#include "observer/emf_veq.h"
#endif
#if M1_EMF_SMO_ENABLE
#include "observer/emf_smo.h"
#endif
#if M1_IDENT_ENABLE
#include "ident_flow.h"
#endif
#if M1_SPEED_IDENT_ENABLE
#include "speed_ident_flow.h"
#include "speed_ident_module.h"
#endif

static motor_context_t s_m1_ctx;

#if M1_EMF_VEQ_ENABLE
static emf_veq_t s_emf_veq;
#endif
#if M1_EMF_SMO_ENABLE
static emf_smo_t s_emf_smo;
#endif

#if M1_IDENT_ENABLE && !M1_IDENT_ID_CAL_BEFORE_STEP && !M1_SPEED_IDENT_ENABLE
/** 0=尚未 ident_flow_init；第一次 JEOC 再 boot（PWM/PI/ADC 零偏已就绪） */
static uint8_t s_ident_booted;
#endif

#if M1_SPEED_IDENT_ENABLE
/** 0=尚未 speed_ident boot；第一次 JEOC 再 boot（与 Bode 同理，避免 init 时 PLL/编码器未稳） */
static uint8_t s_speed_ident_booted;
#endif

#if M1_PLL_ENABLE
static motor_pll_t s_m1_pll;
static float s_pll_theta_mech_prev;
static uint8_t s_pll_theta_mech_prev_valid;
static float s_pll_omega_mech_rpm;
#endif

/** 20 kHz unwrap 机械角 [rad]，外环/VOFA 只读 */
static float s_theta_mech_rad;

/** 速度/位置观测：与 outer_mode 无关，20 kHz 持续更新 dbg */
static void motor_current_update_observation_dbg(void)
{
    dbg.outer_theta_mech_rad = s_theta_mech_rad;
#if M1_PLL_ENABLE
    dbg.outer_omega_mech_rpm = s_pll_omega_mech_rpm;
#endif
}

#if M1_SPEED_LOOP_ENABLE
static uint8_t s_speed_slow_div;
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
#if M1_SPEED_LOOP_ENABLE
    motor_outer_loop_init(&s_m1_ctx);
#endif
#if M1_IDENT_ENABLE || M1_DEADBAND_FLOW_ENABLE || M1_SPEED_IDENT_ENABLE
    s_m1_ctx.mode = M1_CTRL_CURRENT_LOOP;
    s_m1_ctx.id_ref = 0.0f;
    s_m1_ctx.iq_ref = 0.0f;
#if M1_IDENT_ENABLE && !M1_IDENT_ID_CAL_BEFORE_STEP && !M1_SPEED_IDENT_ENABLE
    /* ident HOLD/Bode：推迟到第一次 JEOC。init 时 TIM8 未开、ADC 零偏未完，
     * 此时 boot 会导致上电 Id_ref 一直为 0、open_seq 对不上 60–73。 */
    s_ident_booted = 0u;
    dbg.open_seq_phase = 60u;
#elif M1_SPEED_IDENT_ENABLE
    /* SPEED_IDENT：推迟到第一次 JEOC（见 tick）；此处只占位 */
    s_speed_ident_booted = 0u;
    dbg.open_seq_phase = 220u;
#elif !M1_SPEED_IDENT_ENABLE
    deadband_flow_boot(&s_m1_ctx);
#endif
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

#if M1_PLL_ENABLE
    motor_pll_init(&s_m1_pll,
                   M1_PLL_KP,
                   M1_PLL_KI,
                   M1_PLL_OMEGA_LIMIT_RAD_S,
                   M1_PLL_INTEGRATOR_LIMIT_RAD_S);
    s_pll_theta_mech_prev_valid = 0u;
#endif
#if M1_EMF_VEQ_ENABLE
    emf_veq_init(&s_emf_veq);
#endif
#if M1_EMF_SMO_ENABLE
    emf_smo_init(&s_emf_smo);
#endif
    if (axis->enc != NULL) {
        const uint16_t enc_raw0 = encoder_get_raw(axis->enc);

        s_theta_mech_rad = encoder_get_angle(axis->enc, enc_raw0);
#if M1_PLL_ENABLE
        motor_pll_reset(&s_m1_pll, s_theta_mech_rad);
#endif
    }

#if M1_SPEED_LOOP_ENABLE
#if M1_SPEED_PROFILE_ENABLE
#if !M1_DEADBAND_FLOW_ONE_SHOT && !M1_SPEED_IDENT_ENABLE
    motor_speed_profile_arm(&s_m1_ctx);
    motor_outer_set_mode(&s_m1_ctx, M1_OUTER_SPEED, 0.0f, 0.0f);
#endif
#elif (M1_POS_STEP_TEST_ENABLE || M1_POS_MIT_COMBO_ENABLE) && M1_SPEED_LOOP_BOOT && \
    !M1_SPEED_IDENT_ENABLE && M1_POS_LOOP_ENABLE
#if M1_POS_MIT_COMBO_ENABLE && !M1_POS_MIT_COMBO_POS_ENABLE
    motor_pos_step_request_deferred_boot();
    motor_outer_set_mode(&s_m1_ctx, M1_OUTER_TORQUE, 0.0f, 0.0f);
#else
    motor_pos_step_request_deferred_boot();
    motor_outer_set_mode(&s_m1_ctx, M1_OUTER_POSITION, 0.0f, 0.0f);
#endif
#elif M1_SPEED_REVERSAL_TEST_ENABLE && M1_SPEED_LOOP_BOOT && !M1_SPEED_IDENT_ENABLE
    motor_speed_reversal_arm(&s_m1_ctx);
    motor_outer_set_mode(&s_m1_ctx, M1_OUTER_SPEED, 0.0f, 0.0f);
#elif M1_SPEED_LOOP_BOOT && !M1_SPEED_IDENT_ENABLE && \
    M1_POS_LOOP_ENABLE && M1_POS_LOOP_BOOT
    motor_outer_set_mode(&s_m1_ctx, M1_OUTER_POSITION, 0.0f, 0.0f);
#elif M1_SPEED_LOOP_BOOT && !M1_SPEED_IDENT_ENABLE
    s_m1_ctx.omega_ref = M1_SPEED_REF_RPM_DEFAULT;
    dbg.outer_omega_ref = M1_SPEED_REF_RPM_DEFAULT;
    motor_outer_set_mode(&s_m1_ctx, M1_OUTER_SPEED, 0.0f, 0.0f);
#endif
    s_speed_slow_div = 0u;
#endif
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

float motor_current_get_pll_omega_mech_rpm(void)
{
#if M1_PLL_ENABLE
    return s_pll_omega_mech_rpm;
#else
    return 0.0f;
#endif
}

float motor_current_get_theta_mech_rad(void)
{
    return s_theta_mech_rad;
}

void motor_current_pll_reset_now(void)
{
#if M1_PLL_ENABLE
    motor_pll_reset(&s_m1_pll, s_theta_mech_rad);
    s_pll_omega_mech_rpm = 0.0f;
    s_pll_theta_mech_prev = s_theta_mech_rad;
    s_pll_theta_mech_prev_valid = 0u;
    dbg.pll_omega_mech_rpm = 0.0f;
    dbg.pll_omega_diff_rpm = 0.0f;
    dbg.pll_omega_err_rpm = 0.0f;
    dbg.pll_theta_err_rad = 0.0f;
    dbg.outer_omega_mech_rpm = 0.0f;
#endif
}

#if M1_SPEED_LOOP_ENABLE
void motor_current_outer_set_mode(bsp_axis_t *axis, m1_outer_mode_t mode)
{
    motor_context_t *ctx = motor_current_ctx(axis);

    if (ctx == NULL) {
        return;
    }

    motor_outer_set_mode(ctx, mode, dbg.foc_iq,
                         motor_current_get_pll_omega_mech_rpm());
}

void motor_current_set_omega_ref_rpm(bsp_axis_t *axis, float rpm)
{
    motor_context_t *ctx = motor_current_ctx(axis);

    if (ctx == NULL) {
        return;
    }

    ctx->omega_ref = rpm;
    dbg.outer_omega_ref = rpm;
}

void motor_current_set_theta_ref_rad(bsp_axis_t *axis, float theta_rad)
{
    motor_context_t *ctx = motor_current_ctx(axis);

    if (ctx == NULL) {
        return;
    }

    ctx->theta_ref_rad = theta_rad;
    dbg.outer_theta_ref_rad = theta_rad;
}

void motor_current_arm_position_hold(bsp_axis_t *axis)
{
    motor_context_t *ctx = motor_current_ctx(axis);

    if (ctx == NULL) {
        return;
    }

    motor_outer_arm_position_hold(ctx);
}

void motor_current_set_iq_cmd(bsp_axis_t *axis, float iq_a)
{
    motor_context_t *ctx = motor_current_ctx(axis);

    if (ctx == NULL) {
        return;
    }

    if (iq_a > M1_I_REF_ABS_MAX) {
        iq_a = M1_I_REF_ABS_MAX;
    } else if (iq_a < -M1_I_REF_ABS_MAX) {
        iq_a = -M1_I_REF_ABS_MAX;
    }

    ctx->iq_cmd = iq_a;
}
#endif

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
    volatile uint32_t foc_t0;
    volatile uint32_t t_pre_obs;
    volatile uint32_t t_post_obs;
    volatile uint32_t t_post_svpwm;

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
    dbg.enc_raw = (float)enc_raw;
    s_theta_mech_rad = encoder_get_angle(axis->enc, enc_raw);
    theta = encoder_get_theta_el(axis->enc, enc_raw, ctx->pole_pairs,
                                 encoder_get_theta_el_offset(axis->enc));
#if M1_PLL_ENABLE
    {
        const float theta_mech = s_theta_mech_rad;
        float omega_diff_rpm;

        motor_pll_update(&s_m1_pll, theta_mech, M1_CTRL_TS_S);
        s_pll_omega_mech_rpm = motor_pll_get_omega_mech_rpm(&s_m1_pll);
        dbg.pll_omega_mech_rpm = s_pll_omega_mech_rpm;
        dbg.pll_theta_err_rad = motor_pll_get_last_err(&s_m1_pll);

        if (s_pll_theta_mech_prev_valid) {
            omega_diff_rpm = (theta_mech - s_pll_theta_mech_prev) / M1_CTRL_TS_S;
            omega_diff_rpm = omega_diff_rpm * 60.0f / 6.28318530718f;
        } else {
            omega_diff_rpm = 0.0f;
            s_pll_theta_mech_prev_valid = 1u;
        }
        dbg.pll_omega_diff_rpm = omega_diff_rpm;
        dbg.pll_omega_err_rpm =
            dbg.pll_omega_mech_rpm - dbg.pll_omega_diff_rpm;
        s_pll_theta_mech_prev = theta_mech;

#if M1_PLL_THETA_PARK_ENABLE
        theta = motor_pll_theta_el(&s_m1_pll, ctx->pole_pairs,
                                   encoder_get_theta_el_offset(axis->enc));
#endif
    }
#if M1_SPEED_IDENT_ENABLE
    /* HOLD settle：每拍清 PLL，防止编码器毛刺在开环前把 ω 顶满限幅 */
    if (speed_ident_flow_is_armed() &&
        speed_ident_module_hold_iq_inhibit()) {
        motor_current_pll_reset_now();
    }
#endif
#endif
    motor_current_update_observation_dbg();
#if M1_THETA_NEGATE
    theta_enc_park = -theta;
#else
    theta_enc_park = theta;
#endif
    dbg.enc_theta_el = theta_enc_park;

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
    foc_t0 = *(volatile uint32_t *)&DWT->CYCCNT;
    Clarke_Transform(ia, ib, ic, &i_alpha, &i_beta);

#if M1_IDENT_ENABLE && !M1_IDENT_ID_CAL_BEFORE_STEP && !M1_SPEED_IDENT_ENABLE
    if (s_ident_booted == 0u) {
        ident_flow_init(ctx);
        s_ident_booted = 1u;
    }
    ident_flow_tick(ctx);
#elif M1_SPEED_IDENT_ENABLE
    if (s_speed_ident_booted == 0u) {
        /* 与 Bode 同：PWM/ADC/编码器 SPI 已跑后再 arm，并清 PLL 积分 */
        motor_current_pll_reset_now();
        deadband_flow_boot(ctx);
        speed_ident_flow_init(ctx);
#if M1_EMF_VEQ_ENABLE
        emf_veq_reset(&s_emf_veq);
#endif
#if M1_EMF_SMO_ENABLE
        emf_smo_reset(&s_emf_smo);
#endif
        s_speed_ident_booted = 1u;
    }
    deadband_flow_tick(ctx);
#elif M1_IDENT_ENABLE || M1_DEADBAND_FLOW_ENABLE
    deadband_flow_tick(ctx);
#endif

#if M1_SPEED_LOOP_ENABLE
    if (++s_speed_slow_div >= M1_SPEED_DECIM) {
        s_speed_slow_div = 0u;
        motor_outer_loop_tick(ctx);
    }
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
        if (deadband_id_cal_use_fix_theta() ||
            (deadband_flow_id_cal_active() && !deadband_id_cal_in_iq_probe())) {
            theta_park = deadband_id_cal_target_theta();
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
#if M1_OPEN_UQ_PRE_ID_CAL_ENABLE || M1_OPEN_UD_PRE_ID_CAL_ENABLE
        if ((ctx->mode == M1_CTRL_OBSERVE_ONLY || ctx->mode == M1_CTRL_OPEN_LOOP) &&
            motor_open_sweep_use_fix_theta()) {
            theta_park = M1_OPEN_PRE_ID_LADDER_THETA_EL_RAD;
        }
#endif
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
#if M1_LD_LQ_IDENT_ENABLE
        if (deadband_id_cal_in_ld_lq_ident()) {
#if M1_LD_LQ_IDENT_OPEN_LOOP_ENABLE
            if (ld_lq_ident_inject_active()) {
                ud_out = ld_lq_ident_u_bias_d() + ld_lq_ident_u_inj_d();
                uq_out = ld_lq_ident_u_bias_q() + ld_lq_ident_u_inj_q();
            } else {
                ud_out = ctx->ud_pi;
                uq_out = ctx->uq_pi;
            }
#else
            ud_out = ctx->ud_pi + ld_lq_ident_u_inj_d();
            uq_out = ctx->uq_pi + ld_lq_ident_u_inj_q();
#endif
            ld_lq_ident_integrate(id, iq, ud_out, uq_out);
            break;
        }
#endif
#if M1_ID_CAL_ALIGN_ENABLE && M1_ID_CAL_FIX_THETA_ENABLE
        if (deadband_id_cal_use_align_ud()) {
#if M1_ID_CAL_DUAL_ANGLE_ENABLE
            ud_out = deadband_id_cal_align_ud_v();
#else
            ud_out = M1_ID_CAL_ALIGN_UD_V;
#endif
            uq_out = 0.0f;
        } else if (deadband_id_cal_use_post_ident_hold_ud()) {
            ud_out = M1_ID_CAL_ALIGN_UD_V;
            uq_out = 0.0f;
        } else
#endif
#if M1_LD_LQ_IDENT_ENABLE
        if (deadband_id_cal_use_ld_lq_align_ud()) {
            ud_out = M1_ID_CAL_ALIGN_UD_V;
            uq_out = 0.0f;
        } else
#endif
        if (startup.use_fixed_uq) {
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
#if (M1_OPEN_UQ_PRE_ID_CAL_ENABLE || M1_OPEN_UD_PRE_ID_CAL_ENABLE) && \
    M1_OPEN_PRE_ID_LADDER_ALIGN_ENABLE
        if (motor_open_sweep_in_pre_id_align()) {
            ud_out = M1_OPEN_PRE_ID_LADDER_ALIGN_UD_V;
            uq_out = 0.0f;
            break;
        }
#endif
#if M1_OPEN_UD_PRE_ID_CAL_ENABLE
        if (motor_open_sweep_in_pre_id_ud_ladder()) {
            ud_out = ctx->ud_open;
            uq_out = 0.0f;
            break;
        }
#endif
        ud_out = 0.0f;
        uq_out = ctx->uq_open;
        break;
    default:
        ud_out = 0.0f;
        uq_out = 0.0f;
        break;
    }

    dbg.foc_uq_out = uq_out;
    dbg.foc_ud_out = ud_out;

    t_pre_obs = *(volatile uint32_t *)&DWT->CYCCNT;
#if M1_EMF_VEQ_ENABLE || M1_EMF_SMO_ENABLE
    {
        float omega_mech_rpm = 0.0f;
        float u_alpha;
        float u_beta;

#if M1_PLL_ENABLE
        omega_mech_rpm = s_pll_omega_mech_rpm;
#endif
        /* 与电流环同思路：反 Park 复用本拍 Park 的 sin/cos（θ_park 帧下的 ud/uq） */
        u_alpha = ud_out * cos_el - uq_out * sin_el;
        u_beta = ud_out * sin_el + uq_out * cos_el;

#if M1_EMF_VEQ_ENABLE
        emf_veq_update(&s_emf_veq, i_alpha, i_beta, u_alpha, u_beta,
                       theta_enc_park, omega_mech_rpm);
        dbg.obs_i_alpha = s_emf_veq.i_alpha;
        dbg.obs_i_beta = s_emf_veq.i_beta;
        dbg.obs_u_alpha = s_emf_veq.u_alpha;
        dbg.obs_u_beta = s_emf_veq.u_beta;
        dbg.obs_e_alpha = s_emf_veq.e_alpha;
        dbg.obs_e_beta = s_emf_veq.e_beta;
        dbg.obs_theta_hat = s_emf_veq.theta_hat;
        dbg.obs_theta_err = s_emf_veq.theta_err;
        dbg.obs_emag = s_emf_veq.emag;
        dbg.obs_omega_el = s_emf_veq.omega_el;
        dbg.obs_psi_inst = s_emf_veq.psi_inst;
#endif
#if M1_EMF_SMO_ENABLE
        emf_smo_update(&s_emf_smo, i_alpha, i_beta, u_alpha, u_beta,
                       theta_enc_park, omega_mech_rpm);
        dbg.obs_smo_e_alpha = s_emf_smo.e_alpha;
        dbg.obs_smo_e_beta = s_emf_smo.e_beta;
        dbg.obs_smo_theta_hat = s_emf_smo.theta_hat;
        dbg.obs_smo_theta_err = s_emf_smo.theta_err;
        dbg.obs_smo_emag = s_emf_smo.emag;
#if !M1_EMF_VEQ_ENABLE
        dbg.obs_i_alpha = i_alpha;
        dbg.obs_i_beta = i_beta;
        dbg.obs_u_alpha = u_alpha;
        dbg.obs_u_beta = u_beta;
        dbg.obs_omega_el = s_emf_smo.omega_el;
#endif
#endif
    }
#endif
    t_post_obs = *(volatile uint32_t *)&DWT->CYCCNT;
    g_telem_dbg.obs_delta = t_post_obs - t_pre_obs;

    foc_svpwm_apply_abc(axis, uq_out, ud_out, theta_park, ia, ib, ic, id, iq);
    encoder_kick(axis->enc);
    t_post_svpwm = *(volatile uint32_t *)&DWT->CYCCNT;
    /* FOC 核心 = Clarke→PI + SVPWM/kick，不含观测器 */
    g_telem_dbg.foc_delta = (t_pre_obs - foc_t0) + (t_post_svpwm - t_post_obs);

    telem_bringup_tick();

    {
        volatile uint32_t isr_t1 = *(volatile uint32_t *)&DWT->CYCCNT;

        g_telem_dbg.isr_t0 = isr_t0;
        g_telem_dbg.isr_t1 = isr_t1;
        g_telem_dbg.cyccnt_end = isr_t1;
        g_telem_dbg.isr_delta = isr_t1 - isr_t0;
    }
}
