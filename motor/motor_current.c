/**
 * @file motor_current.c
 * @date 2026-10-06
 * @brief M1 电流环节拍编排：采样、选 Park 角、电流 PI、SVPWM。
 *
 * 本文件负责把一步串起来，并在编码器、I-f、观测器之间选角度。
 * 本文件不填 HAL 句柄，不含 HFI 解调。观测器算法在 motor/observer/。
 * 节拍和中断限制见 motor_current.h 文件头，这里不重复。
 *
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
 * @see docs/电机驱动软件框架——完整架构设计文档_v3.0.md 热路径一节
 */

#include "motor_current.h"

#include <stddef.h>

#include "stm32g474xx.h"

#include "encoder.h"
#include "app_uart_dma_debug.h"
#include "deadband_flow.h"
#include "deadband_id_cal.h"
#include "deadband_module.h"
#include "dbg_monitor.h"
#include "exp_mark.h"
#include "foc_pi.h"
#include "foc_svpwm.h"
#include "ld_lq_ident.h"
#include "motor_foc_loop.h"
#include "motor_open_sweep.h"
#include "motor_outer_loop.h"
#include "motor_phase_binding.h"
#include "motor_cfg.h"
#include "motor_params_m1.h"
#if M1_EXP_FRAMEWORK_ENABLE
#include "experiment/exp_runner.h"
#endif
#if M1_PLL_ENABLE
#include "motor_pll.h"
#endif
#include "motor_startup.h"
#include "motor_if.h"
#include "motor_trig.h"
#include "motor_math.h"
#if M1_EMF_VEQ_ENABLE
#include "observer/emf_veq.h"
#endif
#if M1_EMF_SMO_ENABLE
#include "observer/emf_smo.h"
#endif
#if M1_EMF_PLL_ENABLE
#include "observer/emf_pll.h"
#endif
#if M1_OBS_SOFT_SWITCH_ENABLE
#include "observer/obs_soft_switch.h"
#endif
#if M1_HFI_ENABLE
#include "observer/observer_composite.h"
#endif
#if M1_IDENT_ENABLE
#include "ident_flow.h"
#endif
#if M1_SPEED_IDENT_ENABLE
#include "speed_ident_flow.h"
#include "speed_ident_module.h"
#endif

/** M1 控制上下文。电流环和外环都写这里。 */
static motor_context_t s_m1_ctx;

#if M1_EMF_VEQ_ENABLE
static emf_veq_t s_emf_veq;
#endif
#if M1_EMF_SMO_ENABLE
static emf_smo_t s_emf_smo;
#endif
#if M1_EMF_PLL_ENABLE
static emf_pll_t s_emf_pll;
#endif
#if M1_OBS_SPD_PLL_ENABLE && M1_PLL_ENABLE
/** 观测器速度 PLL。输入是电角，输出经换算得到机械转速。只在观测器速度反馈路径使用。 */
static motor_pll_t s_obs_spd_pll;
/** 1=已经用第一帧电角对齐过；0=下一帧要先对齐，不能当转速用。 */
static uint8_t s_obs_spd_pll_primed;
/** 速度环分频计数。满了才更新一次 PLL。 */
static uint16_t s_obs_spd_div;
/** PLL 换算后的机械转速，单位 rpm。 */
static float s_obs_spd_pll_rpm;
/** 速度反馈低通后的机械转速，单位 rpm。 */
static float s_obs_spd_fb_lpf_rpm;
/** 1=速度反馈低通已经对齐。 */
static uint8_t s_obs_spd_fb_lpf_on;
#endif
#if M1_PLL_ENABLE
/** 外环实际速度反馈，单位 rpm。有感走编码器 PLL；无感速度反馈打开时走观测 PLL。 */
static float s_speed_fb_rpm;
#endif


#if M1_IDENT_ENABLE && !M1_IDENT_ID_CAL_BEFORE_STEP && !M1_SPEED_IDENT_ENABLE
/** 0=辨识流程还未 init。第一拍电流环里再 boot，避免 PWM 未就绪。 */
static uint8_t s_ident_booted;
#endif

#if M1_SPEED_IDENT_ENABLE
/** 0=速度辨识还未 boot。第一拍电流环里再 boot，避免 PLL 未稳。 */
static uint8_t s_speed_ident_booted;
#endif

/** 上电前几拍用实测角重播 PLL，吃掉编码器首帧无效造成的积分器冲顶。 */
#define M1_PLL_PRIME_TICKS   200u

#if M1_PLL_ENABLE
/** 有感机械角 PLL。 */
static motor_pll_t s_m1_pll;
/** 上电重播计数；达到 M1_PLL_PRIME_TICKS 后转正常跟踪。 */
static uint16_t s_pll_prime_ticks;
/** 上一拍机械角，单位 rad。用来 unwrap。 */
static float s_pll_theta_mech_prev;
/** 1=已经有上一拍机械角。 */
static uint8_t s_pll_theta_mech_prev_valid;
/** 编码器 PLL 机械转速，单位 rpm。 */
static float s_pll_omega_mech_rpm;
/** 编码器 PLL 跟踪机械角，单位 rad。位置环可选用它当反馈。 */
static float s_theta_mech_pll_rad;
#endif

/** 20 kHz unwrap 机械角，单位 rad。外环和遥测只读。 */
static float s_theta_mech_rad;


#if M1_SPEED_LOOP_ENABLE
/** 外环分频计数。满 M1_SPEED_DECIM 才调一次外环。 */
static uint8_t s_speed_slow_div;
#endif
#if M1_IF_ENABLE && M1_IF_TO_OBS_ENABLE
/** 1=已经从 I-f 进入观测器，I-f 不再接管。 */
static uint8_t s_if_to_obs_handed;
/** 释放前最后一拍 I-f 速度指令，单位 rpm。 */
static float s_if_omega_cmd_latched;
#if M1_IF_OBS_BLEND_SPEED_ENABLE
/** 1=融合段已经开了弱速度环。 */
static uint8_t s_if_blend_speed_on;
/** 融合段弱速度环分频。 */
static uint8_t s_if_blend_speed_div;
#if M1_IF_OBS_CRUISE_ENABLE
/** 1=巡航已经武装。 */
static uint8_t s_if_cruise_armed;
/** 巡航落稳计时，单位 s。 */
static float s_if_cruise_settle_s;
#endif
#endif
#if M1_IF_OBS_ANGLE_ONLY_ENABLE
/** 只切角时钉死的 Iq，单位 A。避免交角瞬间力矩阶跃。 */
static float s_if_obs_iq_freeze;
#endif
#endif
#if M1_ENC_OPTIONAL_ENABLE || (M1_IF_ENABLE && M1_IF_TO_OBS_ENABLE)
/** 上一拍 Park 角，单位 rad。电流重构和无感兜底用，不要改回吃浮空 θ_enc。 */
static float s_theta_park_last;
#endif
#if M1_HFI_ENABLE && M1_HFI_MOTION_BYPASS_ENABLE
/** RUN 起用上一拍 θ̂ 做扇区重构，单位 rad。 */
static float s_hfi_recon_theta;
/** 1=重构角有效。未进 RUN 前不要用编码器角改电流。 */
static uint8_t s_hfi_recon_theta_ok;
#endif
#if M1_HFI_ENABLE && (M1_OUTER_THETA_FB_SRC == 2)
/** HFI θ̂ 多圈机械角，单位 rad。外环位置反馈。 */
static float s_theta_hfi_mech_rad;
/** 上一拍 HFI 电角，单位 rad。解包用。 */
static float s_theta_hfi_el_prev;
/** 1=已有上一拍电角。 */
static uint8_t s_theta_hfi_el_prev_valid;
/** 1=已在 HFI RUN 就绪后武装签收表。 */
static uint8_t s_sl_signoff_armed;
#endif
#if M1_HFI_ENABLE
/** 本轴观测器 ops。init 后冻结。 */
static const observer_ops_t *s_m1_obs;
/** 本轴注入表。 */
static const observer_inj_ops_t *s_m1_inj;
#endif

#if M1_IF_ENABLE && M1_IF_TO_OBS_ENABLE && M1_IF_OBS_DIR_SEQ_ENABLE
/**
 * @brief 进滑行时清 SMO/PLL，避免 Iq=0 时观测转速假挂。
 */
static void motor_current_dir_seq_try_obs_reset(void)
{
    if (motor_outer_if_obs_dir_seq_consume_obs_reset() == 0u) {
        return;
    }
#if M1_EMF_SMO_ENABLE
    emf_smo_reset(&s_emf_smo);
#endif
#if M1_EMF_PLL_ENABLE
    emf_pll_reset(&s_emf_pll);
#endif
#if M1_OBS_SPD_PLL_ENABLE && M1_PLL_ENABLE
    motor_pll_reset(&s_obs_spd_pll, 0.0f);
    s_obs_spd_pll_primed = 0u;
    s_obs_spd_div = 0u;
    s_obs_spd_pll_rpm = 0.0f;
    s_obs_spd_fb_lpf_rpm = 0.0f;
    s_obs_spd_fb_lpf_on = 0u;
#endif
#if M1_PLL_ENABLE
    s_speed_fb_rpm = 0.0f;
#endif
}

/**
 * @brief 近零后反转再起：清观。软切，I/F 目标。−|ω| 。arm
 * @note 须在 cruise_tick 置位 rearm 之后调用；本。I/F 。tick，下拍起爬坡。
 */
static void motor_current_dir_seq_try_rearm(motor_context_t *ctx)
{
    float abs_tgt;

    if ((ctx == NULL) || (motor_outer_if_obs_dir_seq_consume_rearm() == 0u)) {
        return;
    }

    abs_tgt = M1_IF_TARGET_RPM;
    if (abs_tgt < 0.0f) {
        abs_tgt = -abs_tgt;
    }

#if M1_EMF_SMO_ENABLE
    emf_smo_reset(&s_emf_smo);
#endif
#if M1_EMF_PLL_ENABLE
    emf_pll_reset(&s_emf_pll);
#endif
#if M1_OBS_SOFT_SWITCH_ENABLE
    obs_soft_switch_reset();
#endif
#if M1_OBS_SPD_PLL_ENABLE && M1_PLL_ENABLE
    motor_pll_reset(&s_obs_spd_pll, 0.0f);
    s_obs_spd_pll_primed = 0u;
    s_obs_spd_div = 0u;
    s_obs_spd_pll_rpm = 0.0f;
    s_obs_spd_fb_lpf_rpm = 0.0f;
    s_obs_spd_fb_lpf_on = 0u;
#endif

    s_if_to_obs_handed = 0u;
#if M1_IF_OBS_BLEND_SPEED_ENABLE
    s_if_blend_speed_on = 0u;
    s_if_blend_speed_div = 0u;
#if M1_IF_OBS_CRUISE_ENABLE
    s_if_cruise_armed = 0u;
    s_if_cruise_settle_s = 0.0f;
#endif
#endif

    motor_if_set_target_rpm(-abs_tgt);
    motor_if_arm(ctx);
    ctx->iq_ref = 0.0f;
    ctx->id_ref = 0.0f;
    ctx->omega_ref = 0.0f;
    dbg.outer_omega_ref = 0.0f;
    dbg.open_seq_phase = EXP_MARK_IF_BRINGUP;
}
#endif

#define M1_ACDC_WINDOW_TICKS  10000u

/**
 * @brief 更新 Id/Iq 有效值一类遥测。
 * @param id Id，单位 A。
 * @param iq Iq，单位 A。
 */
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
/**
 * @brief 电流重构用的 Uq，单位 V。
 */
static float motor_current_uq_for_sector(const motor_context_t *ctx)
{
    if (ctx->mode == M1_CTRL_CURRENT_LOOP) {
        return ctx->uq_pi;
    }
    return ctx->uq_open;
}

/**
 * @brief 电流重构用的 Ud，单位 V。
 */
static float motor_current_ud_for_sector(const motor_context_t *ctx)
{
    if (ctx->mode == M1_CTRL_CURRENT_LOOP) {
        return ctx->ud_pi;
    }
    return 0.0f;
}

/**
 * @brief 由占空比和母线电压重构三相电流。
 */
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

/**
 * @brief 初始化 M1 电流环上下文，并按 profile 装配 I-f、HFI 或辨识。
 * @param axis 轴实例。不可为 NULL。
 * @note 辨识和速度 ident 的真正 boot 推迟到第一拍 tick，避免 TIM 与 ADC 零偏未就绪。
 */
void motor_current_init(bsp_axis_t *axis)
{
    if (axis == NULL) {
        return;
    }

    s_m1_ctx.pole_pairs = g_m1_motor_cfg.pole_pairs;
    /* 实验开关初值 = 编译期能力宏；热路径再读 atrb，便于同次上电 A/B。 */
    s_m1_ctx.atrb = 0u;
#if M1_FOC_ROTATION_FF_ENABLE
    s_m1_ctx.atrb |= (uint8_t)M1_ATRB_DECOUP_FF;
#endif
#if M1_FRIC_FF_ENABLE
    s_m1_ctx.atrb |= (uint8_t)M1_ATRB_FRIC_FF;
#endif
    motor_open_sweep_init(&s_m1_ctx);
#if M1_SPEED_LOOP_ENABLE && (!M1_IF_ENABLE || M1_IF_TO_OBS_ENABLE)
    motor_outer_loop_init(&s_m1_ctx);
#endif
#if M1_IF_ENABLE && M1_IF_TO_OBS_ENABLE
    s_if_to_obs_handed = 0u;
    s_if_omega_cmd_latched = 0.0f;
#if M1_IF_OBS_BLEND_SPEED_ENABLE
    s_if_blend_speed_on = 0u;
    s_if_blend_speed_div = 0u;
#if M1_IF_OBS_CRUISE_ENABLE
    s_if_cruise_armed = 0u;
    s_if_cruise_settle_s = 0.0f;
#endif
#endif
#if M1_IF_OBS_ANGLE_ONLY_ENABLE
    s_if_obs_iq_freeze = M1_IF_HANDOFF_IQ_A;
#endif
#endif
#if M1_IF_ENABLE
    s_m1_ctx.mode = M1_CTRL_CURRENT_LOOP;
    s_m1_ctx.id_ref = M1_IF_ID_A;
    s_m1_ctx.iq_ref = M1_IF_IQ_A;
    dbg.open_seq_phase = EXP_MARK_IF_BRINGUP; /* IF bringup marker */
    motor_if_init(&s_m1_ctx);
#elif M1_HFI_ENABLE
    s_m1_ctx.mode = M1_CTRL_CURRENT_LOOP;
    s_m1_ctx.id_ref = 0.0f;
    s_m1_ctx.iq_ref = 0.0f;
    dbg.open_seq_phase = 0u; /* HFI IDLE */
#if M1_EMF_PLL_ENABLE
    observer_bringup(&s_emf_pll);
#else
    observer_bringup(0);
#endif
    s_m1_obs = observer_ops();
    s_m1_inj = observer_inj_ops();
#elif M1_IDENT_ENABLE || M1_DEADBAND_FLOW_ENABLE || M1_SPEED_IDENT_ENABLE
    s_m1_ctx.mode = M1_CTRL_CURRENT_LOOP;
    s_m1_ctx.id_ref = 0.0f;
    s_m1_ctx.iq_ref = 0.0f;
#if M1_IDENT_ENABLE && !M1_IDENT_ID_CAL_BEFORE_STEP && !M1_SPEED_IDENT_ENABLE
    /* ident HOLD/Bode：推迟到第一。JEOC。init 。TIM8 未开、ADC 零偏未完。
     * 此时 boot 会导致上。Id_ref 一直为 0、open_seq 对不。60。3。*/
    s_ident_booted = 0u;
    dbg.open_seq_phase = 60u;
#elif M1_SPEED_IDENT_ENABLE
    /* SPEED_IDENT：推迟到第一。JEOC（见 tick）；此处只占。*/
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
    s_m1_ctx.ia = 0.0f;
    s_m1_ctx.ib = 0.0f;
    s_m1_ctx.ic = 0.0f;
    s_m1_ctx.id = 0.0f;
    s_m1_ctx.iq = 0.0f;
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
#if M1_EMF_PLL_ENABLE
    emf_pll_init(&s_emf_pll);
#endif
#if M1_OBS_SOFT_SWITCH_ENABLE
    obs_soft_switch_init();
#endif
#if M1_OBS_SPD_PLL_ENABLE && M1_PLL_ENABLE
    {
        const float wn = 6.28318530718f * M1_OBS_SPD_PLL_FN_HZ;
        const float kp = 2.0f * M1_OBS_SPD_PLL_ZETA * wn;
        const float ki = wn * wn;
        /* 电角速度限幅 。机械限幅 × 极对。*/
        const float wlim = M1_PLL_OMEGA_LIMIT_RAD_S * (float)s_m1_ctx.pole_pairs;

        motor_pll_init(&s_obs_spd_pll, kp, ki, wlim, wlim);
        s_obs_spd_pll_primed = 0u;
        s_obs_spd_div = 0u;
        s_obs_spd_pll_rpm = 0.0f;
        s_obs_spd_fb_lpf_rpm = 0.0f;
        s_obs_spd_fb_lpf_on = 0u;
    }
#endif
#if M1_PLL_ENABLE
    s_speed_fb_rpm = 0.0f;
#endif
    if (axis->enc != NULL) {
        const uint16_t enc_raw0 = encoder_get_raw(axis->enc);

        s_theta_mech_rad = encoder_get_angle(axis->enc, enc_raw0);
#if M1_PLL_ENABLE
        motor_pll_reset(&s_m1_pll, s_theta_mech_rad);
        s_pll_prime_ticks = 0u;
#endif
    }

#if M1_SPEED_LOOP_ENABLE && (!M1_IF_ENABLE || M1_IF_TO_OBS_ENABLE)
#if M1_OUTER_NEST_ENABLE && (M1_OUTER_EXPT == M1_OUTER_EXPT_SIGNOFF) && \
    M1_SPEED_LOOP_BOOT && !M1_SPEED_IDENT_ENABLE
    motor_outer_set_mode(&s_m1_ctx, M1_OUTER_POSITION, 0.0f, 0.0f);
#if M1_EXP_FRAMEWORK_ENABLE
    exp_runner_init();
    (void)exp_runner_select(EXP_ID_SIGNOFF);
    exp_runner_arm(&s_m1_ctx);
#else
    motor_outer_signoff_arm(&s_m1_ctx);
#endif
#elif M1_SPEED_PROFILE_ENABLE
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
#elif M1_OUTER_NEST_ENABLE && \
    ((M1_OUTER_EXPT == M1_OUTER_EXPT_MIT_HOLD) || \
     (M1_OUTER_EXPT == M1_OUTER_EXPT_MIT_REV)) && \
    M1_SPEED_LOOP_BOOT && !M1_SPEED_IDENT_ENABLE
    motor_outer_set_mode(&s_m1_ctx, M1_OUTER_MIT, 0.0f, 0.0f);
    motor_outer_pos_mini_arm(&s_m1_ctx);
#elif M1_OUTER_NEST_ENABLE && \
    ((M1_OUTER_EXPT == M1_OUTER_EXPT_POS_STEP) || \
     (M1_OUTER_EXPT == M1_OUTER_EXPT_POS_REV)) && \
    M1_SPEED_LOOP_BOOT && !M1_SPEED_IDENT_ENABLE
    motor_outer_set_mode(&s_m1_ctx, M1_OUTER_POSITION, 0.0f, 0.0f);
    motor_outer_pos_mini_arm(&s_m1_ctx);
#elif M1_SPEED_LOOP_BOOT && !M1_SPEED_IDENT_ENABLE && \
    M1_POS_LOOP_ENABLE && M1_POS_LOOP_BOOT
    motor_outer_set_mode(&s_m1_ctx, M1_OUTER_POSITION, 0.0f, 0.0f);
#elif M1_SPEED_LOOP_BOOT && !M1_SPEED_IDENT_ENABLE && !M1_IF_TO_OBS_ENABLE
    s_m1_ctx.omega_ref = M1_SPEED_REF_RPM_DEFAULT;
    dbg.outer_omega_ref = M1_SPEED_REF_RPM_DEFAULT;
    motor_outer_set_mode(&s_m1_ctx, M1_OUTER_SPEED, 0.0f, 0.0f);
#endif
    s_speed_slow_div = 0u;
#endif /* M1_SPEED_LOOP_ENABLE && (!IF || IF_TO_OBS) */
}

/**
 * @brief 取该轴的电机上下文。
 * @param axis 轴实例。可为 NULL，此时返回 NULL。
 * @return 上下文指针；axis 为空时返回 NULL。
 */
motor_context_t *motor_current_ctx(const bsp_axis_t *axis)
{
    if (axis == NULL) {
        return NULL;
    }
    return (motor_context_t *)axis->motor_ctx;
}

/**
 * @brief 切换电流环工作模式。
 * @param axis 轴实例。找不到上下文则直接返回。
 * @param mode 目标模式。切入电流环时会清 PI，并按 profile 重新武装 I-f 或开环启动。
 */
void motor_current_set_mode(bsp_axis_t *axis, m1_ctrl_mode_t mode)
{
    motor_context_t *ctx = motor_current_ctx(axis);

    if (ctx == NULL) {
        return;
    }

    if (mode == M1_CTRL_CURRENT_LOOP && ctx->mode != M1_CTRL_CURRENT_LOOP) {
        motor_foc_loop_pi_reset(ctx);
#if M1_IF_ENABLE
        motor_if_arm(ctx);
#elif M1_STARTUP_ENABLE
        motor_startup_arm(ctx);
#endif
    }

    ctx->mode = mode;
}

/**
 * @brief 写入 Id、Iq 电流指令。
 * @param axis 轴实例。找不到上下文则直接返回。
 * @param id_ref d 轴指令，单位 A。本函数不做绝对值钳位。
 * @param iq_ref q 轴指令，单位 A。本函数不做绝对值钳位。
 */
void motor_current_set_idq_ref(bsp_axis_t *axis, float id_ref, float iq_ref)
{
    motor_context_t *ctx = motor_current_ctx(axis);

    if (ctx == NULL) {
        return;
    }

    ctx->id_ref = id_ref;
    ctx->iq_ref = iq_ref;
}

/**
 * @brief 读取本拍交给外环的机械转速。
 * @return 机械转速，单位 rpm。无 PLL 时返回 0。HFI 速度反馈打开时是观测转速，不是编码器 PLL。
 */
float motor_current_get_pll_omega_mech_rpm(void)
{
#if M1_PLL_ENABLE
    return s_speed_fb_rpm;
#else
    return 0.0f;
#endif
}

/**
 * @brief 读取编码器机械角 PLL 转速。
 * @return 机械转速，单位 rpm。无 PLL 时返回 0。不含 OBS/HFI 速度反馈替换。
 */
float motor_current_get_enc_pll_omega_mech_rpm(void)
{
#if M1_PLL_ENABLE
    return s_pll_omega_mech_rpm;
#else
    return 0.0f;
#endif
}

/**
 * @brief 读取缓存的机械角。
 * @return 多圈连续机械角，单位 rad，上电相对零。来自编码器解包，不是观测角。
 */
float motor_current_get_theta_mech_rad(void)
{
    return s_theta_mech_rad;
}

/**
 * @brief 读取位置环用的位置反馈角。
 * @return 多圈连续机械角，单位 rad。
 *         0=编码器解包，1=编码器 PLL，2=HFI θ̂ 解包。
 */
float motor_current_get_theta_fb_rad(void)
{
#if (M1_OUTER_THETA_FB_SRC == 2) && M1_HFI_ENABLE
    return s_theta_hfi_mech_rad;
#elif (M1_OUTER_THETA_FB_SRC == 1) && M1_PLL_ENABLE
    return s_theta_mech_pll_rad;
#else
    return s_theta_mech_rad;
#endif
}

/**
 * @brief 把编码器 PLL 和速度反馈清零，角度对齐当前机械角。
 * @note 给速度 ident 或模式切换前用。观测器速度 PLL 若编进来也会一起复位。
 */
void motor_current_pll_reset_now(void)
{
#if M1_PLL_ENABLE
    motor_pll_reset(&s_m1_pll, s_theta_mech_rad);
    s_pll_prime_ticks = (uint16_t)M1_PLL_PRIME_TICKS;
    s_theta_mech_pll_rad = s_theta_mech_rad;
    s_pll_omega_mech_rpm = 0.0f;
    s_speed_fb_rpm = 0.0f;
    s_pll_theta_mech_prev = s_theta_mech_rad;
    s_pll_theta_mech_prev_valid = 0u;
    dbg.pll_omega_diff_rpm = 0.0f;
    dbg.pll_omega_err_rpm = 0.0f;
    dbg.pll_theta_err_rad = 0.0f;
#if M1_OBS_SPD_PLL_ENABLE
    motor_pll_reset(&s_obs_spd_pll, 0.0f);
    s_obs_spd_pll_primed = 0u;
    s_obs_spd_div = 0u;
    s_obs_spd_pll_rpm = 0.0f;
    s_obs_spd_fb_lpf_rpm = 0.0f;
    s_obs_spd_fb_lpf_on = 0u;
    dbg.obs_spd_pll_rpm = 0.0f;
#endif
#endif
}

#if M1_SPEED_LOOP_ENABLE
/**
 * @brief 切换外环模式，并用当前 Iq 与转速做无扰交接。
 * @param axis 轴实例。找不到上下文则直接返回。
 * @param mode 外环模式。
 */
void motor_current_outer_set_mode(bsp_axis_t *axis, m1_outer_mode_t mode)
{
    motor_context_t *ctx = motor_current_ctx(axis);

    if (ctx == NULL) {
        return;
    }

    motor_outer_set_mode(ctx, mode, ctx->iq,
                         motor_current_get_pll_omega_mech_rpm());
}

/**
 * @brief 写速度指令。
 * @param axis 轴实例。找不到上下文则直接返回。
 * @param rpm 机械转速指令，单位 rpm。
 */
void motor_current_set_omega_ref_rpm(bsp_axis_t *axis, float rpm)
{
    motor_context_t *ctx = motor_current_ctx(axis);

    if (ctx == NULL) {
        return;
    }

    ctx->omega_ref = rpm;
}

/**
 * @brief 写位置指令。
 * @param axis 轴实例。找不到上下文则直接返回。
 * @param theta_rad 机械角指令，单位 rad。
 */
void motor_current_set_theta_ref_rad(bsp_axis_t *axis, float theta_rad)
{
    motor_context_t *ctx = motor_current_ctx(axis);

    if (ctx == NULL) {
        return;
    }

    ctx->theta_ref_rad = theta_rad;
}

/**
 * @brief 把当前位置锁成位置指令，不改外环模式。
 * @param axis 轴实例。找不到上下文则直接返回。
 */
void motor_current_arm_position_hold(bsp_axis_t *axis)
{
    motor_context_t *ctx = motor_current_ctx(axis);

    if (ctx == NULL) {
        return;
    }

    motor_outer_arm_position_hold(ctx);
}

/**
 * @brief 写力矩通道的 Iq 指令，并按绝对值上限截断。
 * @param axis 轴实例。找不到上下文则直接返回。
 * @param iq_a q 轴指令，单位 A。超过 M1_I_REF_ABS_MAX 则截断。
 */
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

/**
 * @brief 重新武装开环启动序列。
 * @param axis 轴实例。找不到上下文则直接返回。
 * @note 仅 STARTUP 编进来时有效；会清电流 PI。未开启动时为空操作。
 */
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

/**
 * @brief 电流环一步：采样，选 Park 角，电流 PI，SVPWM。
 * @param axis 轴实例。M1 以外当前在入口直接返回。不可为 NULL，且必须已绑编码器和 PWM。
 * @note 无感打开时 Park 用观测角。编码器可选打开时，重构用上一拍控制角，不要改回吃浮空 θ_enc。
 */
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
    float ua_hfi;
    float ub_hfi;
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
    ua_hfi = 0.0f;
    ub_hfi = 0.0f;

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

        if (s_pll_prime_ticks < (uint16_t)M1_PLL_PRIME_TICKS) {
            /* 上电重播：编码器首帧可能无效，先把 PLL 钉在实测角上 */
            s_pll_prime_ticks++;
            motor_pll_reset(&s_m1_pll, theta_mech);
        } else {
            motor_pll_update(&s_m1_pll, theta_mech, M1_CTRL_TS_S);
        }
        s_pll_omega_mech_rpm = motor_pll_get_omega_mech_rpm(&s_m1_pll);
        s_theta_mech_pll_rad = motor_pll_get_theta(&s_m1_pll);
        /*
         * 速度反馈：默认编码器。
         * OBS+速切时必须在 motor_outer_loop_tick 之前换成上一拍观测速，
         * 否则 PI 永远吃有感，拍末覆盖只影。VOFA。
         */
        s_speed_fb_rpm = s_pll_omega_mech_rpm;
#if M1_HFI_ENABLE && M1_HFI_SPEED_FB_ENABLE && M1_HFI_MOTION_BYPASS_ENABLE
        /* 141 纯无感：速度环吃观测 ω（未发布 HFI pll_int，ss≥1 用 SMO）。
         * enc PLL 只留 VOFA ch5。smo_rpm 是上一拍 pub_step。 */
        {
            const float rpm_scale =
                60.0f / (2.0f * 3.14159265f * (float)s_m1_ctx.pole_pairs);

            if (observer_pub_ss() >= 1.0f) {
                s_speed_fb_rpm = observer_pub_smo_rpm();
            } else {
                s_speed_fb_rpm = observer_get_pll_int_el() * rpm_scale;
            }
        }
#endif
#if M1_HFI_ENABLE && (M1_OUTER_THETA_FB_SRC == 2)
        /* 外环 θ_fb：电角差分解包成多圈机械角（用上一拍 θ̂，外环在 pre_park 前）。 */
        {
            const float th_el = observer_get_theta_hat();
            const float pp = (float)s_m1_ctx.pole_pairs;

            if (s_theta_hfi_el_prev_valid == 0u) {
                s_theta_hfi_mech_rad = 0.0f;
                s_theta_hfi_el_prev = th_el;
                s_theta_hfi_el_prev_valid = 1u;
            } else if (pp > 0.0f) {
                s_theta_hfi_mech_rad +=
                    motor_wrap_pi(th_el - s_theta_hfi_el_prev) / pp;
                s_theta_hfi_el_prev = th_el;
            }
        }
#endif
#if M1_OBS_SOFT_SWITCH_ENABLE && M1_OBS_SS_SPEED_SWITCH_ENABLE && M1_OBS_SPD_PLL_ENABLE
        /* BLEND 起与角同步往观测速靠，避。OBS 瞬间 enc→obs 硬切。049 晃速） */
        if (obs_soft_switch_speed_use_obs() != 0u) {
            float omega_obs = s_obs_spd_pll_rpm;

#if M1_OBS_SPD_FB_LPF_HZ > 0
            if (s_obs_spd_fb_lpf_on == 0u) {
#if M1_ENC_OPTIONAL_ENABLE
                /* 无感可选：LPF 初值用 ω̂，勿用死编码。PLL */
                s_obs_spd_fb_lpf_rpm = s_obs_spd_pll_rpm;
#else
                s_obs_spd_fb_lpf_rpm = s_pll_omega_mech_rpm;
#endif
                s_obs_spd_fb_lpf_on = 1u;
            } else {
                const float a = 6.28318530718f * (float)M1_OBS_SPD_FB_LPF_HZ *
                                M1_CTRL_TS_S;
                float alpha = (a > 1.0f) ? 1.0f : a;

                s_obs_spd_fb_lpf_rpm +=
                    alpha * (s_obs_spd_pll_rpm - s_obs_spd_fb_lpf_rpm);
            }
            omega_obs = s_obs_spd_fb_lpf_rpm;
#endif
            if (obs_soft_switch_get_state() == OBS_SS_BLEND) {
#if M1_ENC_OPTIONAL_ENABLE
                /* 可选编码器：融合段也不。enc PLL */
                s_speed_fb_rpm = omega_obs;
#else
                const float blend = obs_soft_switch_get_alpha();
                float b = blend;

                if (b < 0.0f) {
                    b = 0.0f;
                } else if (b > 1.0f) {
                    b = 1.0f;
                }
                s_speed_fb_rpm = s_pll_omega_mech_rpm * (1.0f - b) +
                                 omega_obs * b;
#endif
            } else {
                s_speed_fb_rpm = omega_obs;
            }
        } else {
#if M1_OBS_SPD_FB_LPF_HZ > 0
            s_obs_spd_fb_lpf_on = 0u;
#endif
        }
#endif
        dbg.pll_theta_err_rad = motor_pll_get_last_err(&s_m1_pll);

        if (s_pll_theta_mech_prev_valid) {
            omega_diff_rpm = (theta_mech - s_pll_theta_mech_prev) / M1_CTRL_TS_S;
            omega_diff_rpm = omega_diff_rpm * 60.0f / 6.28318530718f;
        } else {
            omega_diff_rpm = 0.0f;
            s_pll_theta_mech_prev_valid = 1u;
        }
        dbg.pll_omega_diff_rpm = omega_diff_rpm;
        dbg.pll_omega_err_rpm = s_pll_omega_mech_rpm - omega_diff_rpm;
        s_pll_theta_mech_prev = theta_mech;

#if M1_PLL_THETA_PARK_ENABLE
        theta = motor_pll_theta_el(&s_m1_pll, ctx->pole_pairs,
                                   encoder_get_theta_el_offset(axis->enc));
#endif
    }
#if M1_SPEED_IDENT_ENABLE
    /* HOLD settle：每拍清 PLL，防止编码器毛刺在开环前。ω 顶满限幅 */
    if (speed_ident_flow_is_armed() &&
        speed_ident_module_hold_iq_inhibit()) {
        motor_current_pll_reset_now();
    }
#endif
#endif
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
#if M1_ENC_OPTIONAL_ENABLE || (M1_IF_ENABLE && M1_IF_TO_OBS_ENABLE)
        /* 无感/可选编码器：重构用上一拍控制角，勿绑死 θ_enc */
        motor_current_reconstruct_abc(ctx, s_theta_park_last, &ia, &ib, &ic);
#elif M1_HFI_ENABLE && M1_HFI_MOTION_BYPASS_ENABLE
        if (s_hfi_recon_theta_ok != 0u) {
            motor_current_reconstruct_abc(ctx, s_hfi_recon_theta, &ia, &ib, &ic);
        }
#else
        motor_current_reconstruct_abc(ctx, theta_enc_park, &ia, &ib, &ic);
#endif
#endif
    }
    ctx->ia = ia;
    ctx->ib = ib;
    ctx->ic = ic;
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
        /* 。Bode 同：PWM/ADC/编码。SPI 已跑后再 arm，并。PLL 积分 */
        motor_current_pll_reset_now();
        deadband_flow_boot(ctx);
        speed_ident_flow_init(ctx);
#if M1_EMF_VEQ_ENABLE
        emf_veq_reset(&s_emf_veq);
#endif
#if M1_EMF_SMO_ENABLE
        emf_smo_reset(&s_emf_smo);
#endif
#if M1_EMF_PLL_ENABLE
        emf_pll_reset(&s_emf_pll);
#endif
#if M1_OBS_SOFT_SWITCH_ENABLE
        obs_soft_switch_reset();
#endif
#if M1_OBS_SPD_PLL_ENABLE && M1_PLL_ENABLE
        motor_pll_reset(&s_obs_spd_pll, 0.0f);
        s_obs_spd_pll_primed = 0u;
        s_obs_spd_div = 0u;
        s_obs_spd_pll_rpm = 0.0f;
        s_obs_spd_fb_lpf_rpm = 0.0f;
        s_obs_spd_fb_lpf_on = 0u;
#endif
        s_speed_ident_booted = 1u;
    }
    deadband_flow_tick(ctx);
#elif M1_IDENT_ENABLE || M1_DEADBAND_FLOW_ENABLE
    deadband_flow_tick(ctx);
#endif

#if M1_SPEED_LOOP_ENABLE
    /* 注意：s_if_to_obs_handed 是运行时变量，不能写。#if（预处理当成 0。
     * 会把 I/F→OBS 的速度环整段编译掉 。iq_ref 钉死在交接值，Uq 顶满）。*/
    {
        uint8_t run_speed_loop = 1u;

#if M1_IF_ENABLE
#if M1_IF_TO_OBS_ENABLE
        run_speed_loop = s_if_to_obs_handed;
#if M1_IF_OBS_ANGLE_ONLY_ENABLE
        /* 只切角验收：绝不跑速度环，Iq 由冻结值驱。*/
        run_speed_loop = 0u;
#endif
#if M1_IF_OBS_DIR_SEQ_ENABLE
        /* 滑行段：松手，勿。PI 。−Iq 当刹。*/
        if (motor_outer_if_obs_dir_seq_is_coast() != 0u) {
            run_speed_loop = 0u;
            ctx->iq_ref = 0.0f;
            ctx->id_ref = 0.0f;
            ctx->omega_ref = 0.0f;
            dbg.outer_omega_ref = 0.0f;
        }
#endif
#else
        run_speed_loop = 0u;
#endif
#endif
#if M1_HFI_ENABLE && (M1_OUTER_THETA_FB_SRC == 2) && M1_OUTER_NEST_ENABLE && \
    (M1_OUTER_EXPT == M1_OUTER_EXPT_SIGNOFF)
        /* HFI 踢完进 RUN 后再武装签收；此前外环保持 DISABLED，让踢段写 Iq。 */
        if ((s_sl_signoff_armed == 0u) &&
            (observer_speed_run_active() != 0u)) {
            s_sl_signoff_armed = 1u;
            s_theta_hfi_mech_rad = 0.0f;
            motor_outer_set_mode(ctx, M1_OUTER_POSITION, 0.0f, 0.0f);
#if M1_EXP_FRAMEWORK_ENABLE
            exp_runner_init();
            (void)exp_runner_select(EXP_ID_SIGNOFF);
            exp_runner_arm(ctx);
#else
            motor_outer_signoff_arm(ctx);
#endif
        }
#endif
        if (run_speed_loop != 0u) {
            if (++s_speed_slow_div >= M1_SPEED_DECIM) {
                s_speed_slow_div = 0u;
                motor_outer_loop_tick(ctx);
            }
        }
    }
#endif

    if (ctx->mode == M1_CTRL_CURRENT_LOOP) {
#if M1_IDENT_ENABLE || M1_ID_LOCK_CAL_SWEEP
        motor_foc_loop_on_flow_tick(ctx);
#endif
#if M1_IF_ENABLE
        {
            motor_if_step_t if_step = motor_if_tick(ctx, theta_enc_park);

            if (motor_if_is_driving() != 0u) {
                theta_park = if_step.theta_park;
                ctx->id_ref = if_step.id_ref;
#if M1_IF_TO_OBS_ENABLE && M1_IF_OBS_BLEND_SPEED_ENABLE
                /* BLEND 弱速度环已接管 Iq：I/F 只供 θ，勿每拍盖回 4A。233。*/
                if (s_if_blend_speed_on == 0u) {
                    ctx->iq_ref = if_step.iq_ref;
                }
#else
                ctx->iq_ref = if_step.iq_ref;
#endif
#if M1_IF_TO_OBS_ENABLE
                /* 软切 gates 。omega_ref；I/F 段用指令转。*/
                ctx->omega_ref = if_step.omega_cmd_rpm;
                dbg.outer_omega_ref = if_step.omega_cmd_rpm;
                s_if_omega_cmd_latched = if_step.omega_cmd_rpm;
#endif
                startup.theta_park = if_step.theta_park;
                startup.iq_ref = ctx->iq_ref;
                startup.omega_mech_rpm = if_step.omega_meas_rpm;
                startup.use_fixed_uq = 0u;
                startup.uq_out = 0.0f;
                startup.pi_reset = 0u;
                startup.pi_bumpless = 0u;
                startup.uq_prev = 0.0f;
                startup.ud_prev = 0.0f;
                startup.state = M1_STARTUP_CLOSED;
            } else {
#if M1_IF_TO_OBS_ENABLE
                /* 已交给无感：θ 由软切给出，Iq 由速度。*/
#if M1_ENC_OPTIONAL_ENABLE
                theta_park = s_theta_park_last;
#else
                theta_park = theta_enc_park;
#endif
                startup.use_fixed_uq = 0u;
                startup.iq_ref = ctx->iq_ref;
                startup.pi_bumpless = 0u;
                startup.state = M1_STARTUP_CLOSED;
#else
                theta_park = theta_enc_park;
                ctx->iq_ref = 0.0f;
                startup.iq_ref = 0.0f;
                startup.use_fixed_uq = 0u;
                startup.state = M1_STARTUP_CLOSED;
#endif
            }
            dbg.startup_state = (uint8_t)if_step.state;
            dbg.startup_omega_mech_rpm = if_step.omega_meas_rpm;
            dbg.if_omega_cmd_rpm = if_step.omega_cmd_rpm;
            dbg.if_theta_err_rad = if_step.theta_err_rad;
        }
#else
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
        dbg.if_omega_cmd_rpm = 0.0f;
        dbg.if_theta_err_rad = 0.0f;
#endif /* M1_IF_ENABLE */
    } else {
        theta_park = theta_enc_park;
        dbg.startup_state = (uint8_t)M1_STARTUP_CLOSED;
        dbg.startup_omega_mech_rpm = 0.0f;
        dbg.if_omega_cmd_rpm = 0.0f;
        dbg.if_theta_err_rad = 0.0f;
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
#if M1_OBS_SOFT_SWITCH_ENABLE && M1_EMF_PLL_ENABLE
    /* 用上一。PLL θ̂；本拍观测器。Park/SVPWM 之后更新 */
    if (s_emf_pll.primed != 0u) {
        float omega_enc_ss = 0.0f;
        float omega_obs_ss = s_obs_spd_pll_rpm; /* 上一拍观测速，。OBS 掉速门。*/
        float theta_ss_base = theta_enc_park;

#if M1_PLL_ENABLE
#if M1_ENC_OPTIONAL_ENABLE
        /* 可选编码器：软切不。ω_enc；有感监督只。VOFA */
        omega_enc_ss = 0.0f;
#else
        omega_enc_ss = s_pll_omega_mech_rpm;
#endif
#endif
#if M1_IF_ENABLE && M1_IF_TO_OBS_ENABLE
        /* I/F→OBS：融合底角用 θ_if，避免切入跳到编码器。*/
        if (motor_if_is_driving() != 0u) {
            theta_ss_base = theta_park;
        }
#if M1_ENC_OPTIONAL_ENABLE
        else {
            /* I/F 已释放：底角用上一拍控制角，勿回落垃圾 θ_enc */
            theta_ss_base = s_theta_park_last;
        }
#endif
#endif
        {
            float theta_hat_park = s_emf_pll.theta_hat;
            float ss_theta_err = s_emf_pll.theta_err;

#if M1_IF_ENABLE && M1_IF_TO_OBS_ENABLE
            /* 门限角差。θ̂−θ_if，不依赖编码器（拔掉编码器仍可判。*/
            if (motor_if_is_driving() != 0u) {
                float e = theta_hat_park - theta_ss_base;

                while (e > 3.14159265359f) {
                    e -= 6.28318530718f;
                }
                while (e < -3.14159265359f) {
                    e += 6.28318530718f;
                }
                ss_theta_err = e;
            }
#endif
            theta_park = obs_soft_switch_apply(theta_ss_base,
                                               theta_hat_park,
                                               ss_theta_err,
                                               s_emf_pll.emag,
                                               omega_enc_ss,
                                               omega_obs_ss,
                                               ctx->omega_ref,
                                               ctx->iq,
                                               M1_CTRL_TS_S);
        }
#if M1_IF_ENABLE && M1_IF_TO_OBS_ENABLE
        /* 进入 OBS：释。I/F。ANGLE_ONLY=固定 Iq；否则速度。bumpless 接管 */
        if ((s_if_to_obs_handed == 0u) &&
            (obs_soft_switch_speed_on_obs() != 0u)) {
            const float omega_meas = s_speed_fb_rpm;
            float omega_hold;

            s_if_to_obs_handed = 1u;
            motor_if_release();

            /* HOLD：优先钉 I/F 指令速，避免。BLEND 飞车速当成目标（2249。*/
#if M1_IF_OBS_HOLD_SPEED_ENABLE && M1_IF_OBS_HOLD_IF_CMD_ENABLE
            omega_hold = s_if_omega_cmd_latched;
            /* 。|ω| 判有效：负速时 latched<1 不能当“未就绪。*/
            if ((omega_hold > -1.0f) && (omega_hold < 1.0f)) {
                omega_hold = omega_meas;
            }
#elif M1_IF_OBS_HOLD_SPEED_ENABLE
            omega_hold = omega_meas;
#else
            omega_hold = M1_IF_OBS_SPEED_REF_RPM;
#endif
            ctx->omega_ref = omega_hold;
            dbg.outer_omega_ref = omega_hold;
#if M1_IF_OBS_ANGLE_ONLY_ENABLE
            /* 只切角：。Iq，不启动速度外环 */
            s_if_obs_iq_freeze = M1_IF_HANDOFF_IQ_A;
            ctx->iq_ref = s_if_obs_iq_freeze;
            ctx->id_ref = M1_IF_ID_A;
            dbg.open_seq_phase = EXP_MARK_IF_OBS_ANGLE_ONLY; /* IF→OBS angle-only */
#else
#if M1_IF_OBS_BLEND_SPEED_ENABLE
            if (s_if_blend_speed_on != 0u) {
                /* BLEND 已开外环：延续当。iq_ref，只。ω_IF */
                motor_outer_sync_speed_boot(ctx, ctx->iq_ref, omega_meas);
                motor_outer_set_omega_ramp_rpm(omega_hold);
                ctx->omega_ref = omega_hold;
                dbg.outer_omega_ref = omega_hold;
                dbg.open_seq_phase = EXP_MARK_IF_OBS_BLEND_SPEED; /* IF→OBS, speed from BLEND */
            } else
#endif
            {
                const float iq_boot = M1_IF_OBS_HANDOFF_IQ_BOOT_A;

                ctx->iq_ref = iq_boot;
                /* bumpless：ref=ω_hold，fb=实测；斜坡起点钉。hold */
                motor_outer_set_mode(ctx, M1_OUTER_SPEED, iq_boot, omega_meas);
                motor_outer_sync_speed_boot(ctx, iq_boot, omega_meas);
                motor_outer_set_omega_ramp_rpm(omega_hold);
                ctx->omega_ref = omega_hold;
                dbg.outer_omega_ref = omega_hold;
#if M1_IF_OBS_EW_CLAMP_ENABLE
                motor_outer_if_obs_ew_guard_arm();
#endif
#if M1_IF_OBS_SOFT_BRAKE_ENABLE
                motor_outer_if_obs_soft_brake_arm(ctx);
                /* 浅刹后再。bumpless：防 slew 仍停。I/F 。+Iq */
                motor_outer_sync_speed_boot(ctx, ctx->iq_ref, omega_meas);
#endif
                dbg.open_seq_phase = EXP_MARK_IF_OBS_HANDED; /* IF→OBS handed, ω_ref=ω_IF */
            }
#endif
        }
#if M1_IF_OBS_CRUISE_ENABLE && M1_IF_TO_OBS_ENABLE
        /* 速切站稳 。三步巡航（①ref ②Iq ③PI。*/
        if (obs_soft_switch_speed_use_obs() != 0u) {
            if (s_if_cruise_armed == 0u) {
                s_if_cruise_settle_s += M1_CTRL_TS_S;
                if (s_if_cruise_settle_s >= M1_IF_OBS_CRUISE_SETTLE_S) {
                    motor_outer_if_obs_cruise_arm(ctx, s_speed_fb_rpm);
                    s_if_cruise_armed = 1u;
                }
            } else {
                motor_current_dir_seq_try_obs_reset();
                motor_outer_if_obs_cruise_tick(ctx, s_speed_fb_rpm, M1_CTRL_TS_S);
            }
#if M1_IF_OBS_DIR_SEQ_ENABLE
            motor_current_dir_seq_try_rearm(ctx);
#endif
        } else {
            s_if_cruise_settle_s = 0.0f;
        }
#endif
#if M1_IF_OBS_ANGLE_ONLY_ENABLE
        /* 已切角：每拍钉死 Iq，防止其它路径改。*/
        if (s_if_to_obs_handed != 0u) {
            ctx->iq_ref = s_if_obs_iq_freeze;
            ctx->id_ref = M1_IF_ID_A;
        }
#endif
        /* BLEND：KEEP_IF_IQ 强制拖动电流；BLEND_SPEED 则本段外环写 Iq */
        if ((motor_if_is_driving() != 0u) &&
            (obs_soft_switch_get_state() == OBS_SS_BLEND)) {
#if M1_IF_OBS_BLEND_SPEED_ENABLE && !M1_IF_OBS_ANGLE_ONLY_ENABLE
            {
                const float omega_meas_b = s_speed_fb_rpm;
                float omega_hold_b = s_if_omega_cmd_latched;

                /* 。OBS 交接：|ω|<1 才回退实测（兼容反向） */
                if ((omega_hold_b > -1.0f) && (omega_hold_b < 1.0f)) {
                    omega_hold_b = omega_meas_b;
                }
                if (s_if_blend_speed_on == 0u) {
                    /* 转子系：反向驱动 −Iq；勿。I/F 。+Iq 否则浅刹。max→失。*/
                    const float iq_boot = M1_IF_OBS_SPEED_IQ_BOOT_A;

                    s_if_blend_speed_on = 1u;
                    s_if_blend_speed_div = 0u;
                    ctx->omega_ref = omega_hold_b;
                    dbg.outer_omega_ref = omega_hold_b;
                    ctx->iq_ref = iq_boot;
                    ctx->id_ref = M1_IF_ID_A;
                    motor_outer_set_mode(ctx, M1_OUTER_SPEED, iq_boot, omega_meas_b);
                    motor_outer_sync_speed_boot(ctx, iq_boot, omega_meas_b);
                    motor_outer_set_omega_ramp_rpm(omega_hold_b);
#if M1_IF_OBS_EW_CLAMP_ENABLE
                    motor_outer_if_obs_ew_guard_arm();
#endif
#if M1_IF_OBS_SOFT_BRAKE_ENABLE
                    motor_outer_if_obs_soft_brake_arm(ctx);
                    motor_outer_sync_speed_boot(ctx, ctx->iq_ref, omega_meas_b);
#endif
                    dbg.open_seq_phase = EXP_MARK_IF_OBS_BLEND_WEAK; /* BLEND + weak speed */
                }
                ctx->omega_ref = omega_hold_b;
                dbg.outer_omega_ref = omega_hold_b;
                motor_outer_set_omega_ramp_rpm(omega_hold_b);
                ctx->id_ref = M1_IF_ID_A;
                if (++s_if_blend_speed_div >= M1_SPEED_DECIM) {
                    s_if_blend_speed_div = 0u;
                    motor_outer_loop_tick(ctx);
                }
            }
#elif M1_IF_OBS_ANGLE_ONLY_ENABLE || M1_IF_OBS_BLEND_KEEP_IF_IQ
            ctx->iq_ref = M1_IF_IQ_A;
            ctx->id_ref = M1_IF_ID_A;
#else
            const float a = obs_soft_switch_get_alpha();
            float a_clamped = a;

            if (a_clamped < 0.0f) {
                a_clamped = 0.0f;
            } else if (a_clamped > 1.0f) {
                a_clamped = 1.0f;
            }
            ctx->iq_ref = M1_IF_IQ_A +
                (M1_IF_HANDOFF_IQ_A - M1_IF_IQ_A) * a_clamped;
            ctx->id_ref = M1_IF_ID_A;
#endif
        }
#endif
    } else {
#if !(M1_IF_ENABLE && M1_IF_TO_OBS_ENABLE)
        theta_park = theta_enc_park;
#else
        if (motor_if_is_driving() == 0u) {
#if M1_ENC_OPTIONAL_ENABLE
            theta_park = s_theta_park_last;
#else
            theta_park = theta_enc_park;
#endif
        }
        /* else：保。I/F θ，等 PLL primed */
#endif
    }
    dbg.obs_ss_state = (float)obs_soft_switch_get_state();
    dbg.obs_ss_alpha = obs_soft_switch_get_alpha();
    dbg.obs_ss_spd_on = (float)obs_soft_switch_speed_use_obs();
#endif
#if M1_HFI_ENABLE
    s_m1_obs->pre_park(theta_enc_park, M1_CTRL_TS_S);
    theta_park = s_m1_obs->get_theta();
#if M1_HFI_MOTION_BYPASS_ENABLE
    if (observer_get_stage() == OBS_STAGE_RUN) {
        s_hfi_recon_theta = theta_park;
        s_hfi_recon_theta_ok = 1u;
    }
#endif
#ifndef M1_HFI_QKICK_AFTER_LOCK_ENABLE
#define M1_HFI_QKICK_AFTER_LOCK_ENABLE 0
#endif
#if M1_HFI_QKICK_AFTER_LOCK_ENABLE
    if (observer_consume_pi_reset() != 0u) {
        /* 踢时只清 Iq PI，Id 积分保持 */
        foc_pi_reset(&ctx->pi_iq);
        ctx->uq_pi = 0.0f;
    }
#endif
#if M1_HFI_MOTION_BYPASS_ENABLE && M1_SPEED_LOOP_ENABLE
    {
#if (M1_OUTER_THETA_FB_SRC == 2) && M1_OUTER_NEST_ENABLE && \
    (M1_OUTER_EXPT == M1_OUTER_EXPT_SIGNOFF)
        /*
         * 无感位控：签收武装后外环管模式/Iq；武装前仍让 HFI 写踢段电流。
         * 禁止 speed_run_active 把外环抢成 ±1500 速度阶梯。
         */
        if (motor_outer_signoff_is_armed() == 0u) {
            ctx->id_ref = observer_get_id_ref();
            ctx->iq_ref = observer_get_iq_ref();
            ctx->omega_ref = 0.0f;
            dbg.outer_omega_ref = 0.0f;
            if (ctx->outer_mode != M1_OUTER_DISABLED) {
                motor_outer_set_mode(ctx, M1_OUTER_DISABLED, 0.0f, 0.0f);
            }
        }
        observer_set_omega_ff_el(0.0f);
#else
        if (observer_speed_run_active() != 0u) {
            const float rpm_cmd = observer_get_speed_ref_rpm();

            ctx->omega_ref = rpm_cmd;
            dbg.outer_omega_ref = rpm_cmd;
            if (ctx->outer_mode != M1_OUTER_SPEED) {
                /* ω* 已写成目标。bumpless 若看见它，积分会预成 -Kp·目标。冷启动。0 交接。*/
                ctx->omega_ref = 0.0f;
                motor_outer_set_mode(ctx, M1_OUTER_SPEED, 0.0f, 0.0f);
                ctx->omega_ref = rpm_cmd;
                dbg.outer_omega_ref = rpm_cmd;
            }
        } else {
            /* 速度环关：由 QKICK/HFI 写 id_ref/iq_ref/omega_ref */
            ctx->id_ref = observer_get_id_ref();
            ctx->iq_ref = observer_get_iq_ref();
            ctx->omega_ref = observer_get_speed_ref_rpm();
            dbg.outer_omega_ref = ctx->omega_ref;
            if (ctx->outer_mode != M1_OUTER_DISABLED) {
                motor_outer_set_mode(ctx, M1_OUTER_DISABLED, 0.0f, 0.0f);
            }
        }
        /* 141：OMEGA_FF_SRC=0，禁止 enc/指令转速进 HFI 前馈 */
        observer_set_omega_ff_el(0.0f);
#endif
    }
#endif
#endif
    dbg.foc_theta_el = theta_park;
#if M1_ENC_OPTIONAL_ENABLE || (M1_IF_ENABLE && M1_IF_TO_OBS_ENABLE)
    s_theta_park_last = theta_park;
#endif

    motor_trig_sincos(theta_park, &cos_el, &sin_el);
    Park_Transform_sc(i_alpha, i_beta, sin_el, cos_el, &id, &iq);
    ctx->id = id;
    ctx->iq = iq;
    dbg.foc_id_lpf = id; /* CURRENT_LOOP + ID_PI_LPF 时下面会覆盖 */

#if M1_HFI_ENABLE
    s_m1_obs->post_park(id, iq, i_alpha, i_beta);
#if M1_HFI_IQ_AUTH_ENABLE
    /* 解调后更新权威：限速环 Iq，并。PI 上下限防积分顶满 */
    if (observer_speed_run_active() != 0u) {
        const float lim = observer_get_iq_auth_abs();

        if (ctx->iq_ref > lim) {
            ctx->iq_ref = lim;
        } else if (ctx->iq_ref < -lim) {
            ctx->iq_ref = -lim;
        }
        ctx->pi_speed.out_max = lim;
        ctx->pi_speed.out_min = -lim;
        ctx->pi_speed.int_max = lim;
        ctx->pi_speed.int_min = -lim;
        if (ctx->pi_speed.integrator > lim) {
            ctx->pi_speed.integrator = lim;
        } else if (ctx->pi_speed.integrator < -lim) {
            ctx->pi_speed.integrator = -lim;
        }
    } else {
        ctx->pi_speed.out_max = M1_SPEED_PI_OUT_MAX;
        ctx->pi_speed.out_min = M1_SPEED_PI_OUT_MIN;
        ctx->pi_speed.int_max = M1_SPEED_PI_INT_MAX;
        ctx->pi_speed.int_min = M1_SPEED_PI_INT_MIN;
    }
#endif
#endif

#if M1_HFI_ENABLE
    /*
     * 无扰：id* = (1-soft)·id → 误差 = -soft·id；
     * soft=0 时误差为 0；soft→1 → id*→0。须在 foc_loop 之前。
     * 141：ID_ON_FROM_RUN → HAND_ID_WITH_VH 常开。
     */
    if (observer_id_pi_bypass() == 0u) {
        const float soft = observer_id_pi_soft_scale();

        if (soft < 1.0f) {
            ctx->id_ref = (1.0f - soft) * id;
        } else {
            ctx->id_ref = 0.0f;
        }
    }
#endif

    motor_foc_loop_dbg_id_ref(ctx);
    motor_current_update_acdc(id, iq);

    if (ctx->mode == M1_CTRL_CURRENT_LOOP) {
        float id_for_pi = id;
        dbg.foc_id_lpf = id;
#if !M1_IF_ENABLE
        motor_startup_finish_tick(ctx, iq, &startup);
#endif
        motor_foc_loop_tick(ctx, id_for_pi, iq, &startup, theta_enc_park);
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
#if M1_HFI_ENABLE
            /* 旁路：清 Id。放行后：Ud*=soft；OVERLAP 不清积分（id* 无扰）。*/
            if (observer_id_pi_bypass() != 0u) {
                foc_pi_reset(&ctx->pi_id);
                ctx->ud_pi = 0.0f;
                dbg.foc_ud_pi = 0.0f;
                ud_out = deadband_service_ud_inject(id);
            } else {
                float soft = observer_id_pi_soft_scale();
                float ud_db = deadband_service_ud_inject(id);

                /* HAND_ID_WITH_VH：soft 爬坡期间不清 Id 积分 */
                ud_out = ud_db + soft * ctx->ud_pi;
                dbg.foc_ud_pi = soft * ctx->ud_pi;
            }
#endif
        }
#if M1_HFI_ENABLE
        {
            float ud_ov;
            float uq_ov;
            float ud_inj;
            float uq_inj;

            if ((s_m1_inj != 0) &&
                (s_m1_inj->override_voltage(&ud_ov, &uq_ov) != 0u)) {
                ud_out = ud_ov;
                uq_out = uq_ov;
                /*
                 * IDLE/DONE 等强制电压时 PI 输出被丢弃；若不卸积分，
                 * 再进 RUN 会带着饱和 Ud_pi（C4k 1300：IDLE~13.8V→PRE 解调崩）。
                 */
                if ((ud_ov == 0.0f) && (uq_ov == 0.0f)) {
                    foc_pi_reset(&ctx->pi_id);
                    foc_pi_reset(&ctx->pi_iq);
                    ctx->ud_pi = 0.0f;
                    ctx->uq_pi = 0.0f;
                    dbg.foc_ud_pi = 0.0f;
                    dbg.foc_uq_pi = 0.0f;
                }
            } else if (s_m1_inj != 0) {
                s_m1_inj->get_inj(&ud_inj, &uq_inj);
                ud_out += ud_inj;
                uq_out += uq_inj;
                /* 进桥的载波，供 SMO 减 Vh；与本拍 Park 同框 */
                ua_hfi = ud_inj * cos_el - uq_inj * sin_el;
                ub_hfi = ud_inj * sin_el + uq_inj * cos_el;
                {
                    float ua_inj;
                    float ub_inj;

                    /* ±Vh 沿 θ̂ 在定子上；旋回本拍 Park 再进 SVPWM */
                    s_m1_inj->get_inj_ab(&ua_inj, &ub_inj);
                    ud_out += ua_inj * cos_el + ub_inj * sin_el;
                    uq_out += -ua_inj * sin_el + ub_inj * cos_el;
                    ua_hfi += ua_inj;
                    ub_hfi += ub_inj;
                }
            }
        }
#endif
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

#if M1_HFI_ENABLE
    {
        observer_view_t tv;
        const float rpm_scale =
            60.0f / (2.0f * 3.14159265f * (float)s_m1_ctx.pole_pairs);

        /* T3：此处 harvest snap；其后至 observer_telem_publish 勿再改 snap 源 */
        observer_read_view(&tv);
        dbg.hfi_theta_cmd = tv.theta_cmd; /* ch1: θ_cmd [rad] */
        dbg.hfi_theta_hat = tv.theta_hat;
        dbg.hfi_theta_err = tv.theta_err;
        dbg.hfi_eps = tv.eps;
        dbg.hfi_di_q = tv.di_q;
        dbg.hfi_di_d = tv.di_d;
        dbg.hfi_x_raw = tv.x_raw;
        dbg.hfi_y_raw = tv.y_raw;
        dbg.hfi_vh_sign = tv.vh_sign;
        dbg.hfi_stage = (float)tv.stage + tv.eps_dead;
        dbg.hfi_lock = (float)tv.lock;
        /* 速度观测 = 积分项；角度仍由 Kp·eps + I 推进 */
        dbg.hfi_omega_rpm = tv.pll_int_el * rpm_scale;
        /* 台架：ch9 = eps_d + 0.1·flip + 0.01·axis_ok */
        dbg.hfi_omega_trim_rpm =
            tv.eps_d + 0.1f * tv.axis_flip_n +
            ((tv.axis_ok != 0u) ? 0.01f : 0.0f);
        dbg.hfi_qkick_verdict = tv.pll_int_el * rpm_scale;
        dbg.hfi_ipd_phase = (float)tv.qkick_phase;
        dbg.hfi_ipd_pulse_ud = tv.ipd_pulse_ud;
        dbg.hfi_qkick_seed = tv.qkick_seed;
        dbg.hfi_qkick_dth_deg = tv.qkick_dth * (180.0f / 3.14159265f);
    }
#if (M1_OUTER_THETA_FB_SRC == 2) && M1_OUTER_NEST_ENABLE && \
    (M1_OUTER_EXPT == M1_OUTER_EXPT_SIGNOFF)
    /*
     * 武装前用 HFI stage 看锁相；武装中/停环打点（255/252）后都不要盖掉。
     * 否则跑完那一拍 255 马上被写成 7，录波看起来像中途崩了。
     */
    if ((motor_outer_signoff_is_armed() == 0u) &&
        (dbg.open_seq_phase != EXP_MARK_SIGN_DONE) &&
        (dbg.open_seq_phase != EXP_MARK_SIGN_GUARD)) {
        dbg.open_seq_phase = (uint8_t)observer_get_stage();
    }
#else
    dbg.open_seq_phase = (uint8_t)observer_get_stage();
#endif
#endif

    t_pre_obs = *(volatile uint32_t *)&DWT->CYCCNT;
#if M1_EMF_VEQ_ENABLE || M1_EMF_SMO_ENABLE || M1_EMF_PLL_ENABLE
    {
        float omega_mech_rpm = 0.0f;
        float u_alpha;
        float u_beta;
        float theta_obs_ref = theta_enc_park;
        uint8_t smo_hold = 0u;

#if M1_PLL_ENABLE
        omega_mech_rpm = s_speed_fb_rpm;
#endif
#if M1_HFI_ENABLE && M1_HFI_MOTION_BYPASS_ENABLE && M1_EMF_SMO_ENABLE && \
    M1_EMF_PLL_ENABLE
        /* 旁路相对 θ̂_hfi。速度环未接管前每个周期清掉，避免零速假转速和编码器初值。*/
        theta_obs_ref = observer_get_theta_hat();
        /* 并行观测：只在极性翻面清状态，HFI 段继续跑 SMO */
        if (observer_take_polarity_flip() != 0u) {
            smo_hold = 1u;
            emf_smo_reset(&s_emf_smo);
            observer_emf_reset();
            observer_smo_w_ma_reset();
            dbg.obs_pll_theta_hat = 0.0f;
            dbg.obs_pll_theta_err = 0.0f;
            dbg.obs_pll_omega_el = 0.0f;
            dbg.obs_spd_pll_rpm = 0.0f;
            dbg.obs_spd_rpm_err = 0.0f;
        }
#endif
        if (smo_hold != 0u) {
            /* 本拍不更。*/
        } else {
        /* 与电流环同思路：反 Park 复用本拍 Park 。sin/cos（θ_park 帧下。ud/uq。*/
        u_alpha = ud_out * cos_el - uq_out * sin_el;
        u_beta = ud_out * sin_el + uq_out * cos_el;
        /* 桥上仍叠 Vh；磁链/SMO 只吃基波 */
        u_alpha -= ua_hfi;
        u_beta -= ub_hfi;

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
        {
            const uint8_t lpf_band = emf_smo_lpf_sched_update(omega_mech_rpm);

            emf_smo_update(&s_emf_smo, i_alpha, i_beta, u_alpha, u_beta,
                           theta_obs_ref, omega_mech_rpm);
            dbg.obs_smo_e_alpha = s_emf_smo.e_alpha;
            dbg.obs_smo_e_beta = s_emf_smo.e_beta;
            dbg.obs_smo_theta_hat = s_emf_smo.theta_hat;
            dbg.obs_smo_theta_err = s_emf_smo.theta_err;
            dbg.obs_smo_emag = s_emf_smo.emag;
            dbg.obs_smo_lpf_hz = emf_smo_get_lpf_hz();
            dbg.obs_smo_lpf_band = (float)lpf_band;
        }
#if !M1_EMF_VEQ_ENABLE
        dbg.obs_i_alpha = i_alpha;
        dbg.obs_i_beta = i_beta;
        dbg.obs_u_alpha = u_alpha;
        dbg.obs_u_beta = u_beta;
        dbg.obs_omega_el = s_emf_smo.omega_el;
#endif
#endif
#if M1_EMF_PLL_ENABLE
#if M1_EMF_PLL_USE_SMO
        observer_emf_update(s_emf_smo.e_alpha, s_emf_smo.e_beta,
                            theta_obs_ref, M1_CTRL_TS_S);
#else
        observer_emf_update(s_emf_veq.e_alpha, s_emf_veq.e_beta,
                            theta_obs_ref, M1_CTRL_TS_S);
#endif
        dbg.obs_pll_theta_hat = observer_emf_theta_hat();
        dbg.obs_pll_theta_err = observer_emf_theta_err();
        dbg.obs_pll_omega_el = observer_emf_omega_el();
        dbg.obs_pll_pd = observer_emf_last_pd();
#if M1_HFI_ENABLE && M1_HFI_MOTION_BYPASS_ENABLE && M1_EMF_PLL_ENABLE
        {
            const float emf_th = observer_emf_theta_hat();
            const float emf_w = observer_emf_omega_el();
            float dth = emf_th - theta_obs_ref;
            const float rpm_scale =
                60.0f / (6.28318530718f * (float)s_m1_ctx.pole_pairs);

            while (dth > 3.14159265f) {
                dth -= 6.2831853f;
            }
            while (dth < -3.14159265f) {
                dth += 6.2831853f;
            }
            /* 旁路角差 = θ_smo 。θ_hfi。速度环不读这个量。*/
            dbg.obs_pll_theta_err = dth;
            dbg.obs_spd_rpm_err = emf_w * rpm_scale;
            dbg.obs_spd_pll_rpm = observer_smo_w_ma_step(dbg.obs_spd_rpm_err);
            observer_pub_step(emf_th, emf_w, dth);
            dbg.obs_ss_spd_on = observer_pub_ss();
            dbg.obs_spd_rpm_err = observer_pub_smo_rpm();
        }
#endif
#if M1_OBS_SPD_PLL_ENABLE && M1_PLL_ENABLE
        /* θ̂ 。速度环同。PLL @ 2 kHz（省 ISR）；有感段只预热，OBS 时进速度。*/
        if (s_emf_pll.primed != 0u) {
            const float pp = (float)s_m1_ctx.pole_pairs;

            if (s_obs_spd_pll_primed == 0u) {
                motor_pll_reset(&s_obs_spd_pll, s_emf_pll.theta_hat);
                s_obs_spd_pll_primed = 1u;
                s_obs_spd_div = 0u;
            }
            if (++s_obs_spd_div >= M1_SPEED_DECIM) {
                float rpm_obs;

                s_obs_spd_div = 0u;
                motor_pll_update(&s_obs_spd_pll, s_emf_pll.theta_hat,
                                 M1_SPEED_TS_S);
                rpm_obs = motor_pll_get_omega_mech(&s_obs_spd_pll) * 60.0f /
                          (6.28318530718f * pp);
                s_obs_spd_pll_rpm = rpm_obs;
                dbg.obs_spd_pll_rpm = rpm_obs;
                dbg.obs_spd_pll_err_rad =
                    motor_pll_get_last_err(&s_obs_spd_pll);
            }
        }
#if M1_OBS_SOFT_SWITCH_ENABLE && M1_OBS_SS_SPEED_SWITCH_ENABLE
        /* 与速度环反馈一致（先角后速时 OBS 初期仍为编码器） */
        dbg.outer_omega_mech_rpm = s_speed_fb_rpm;
#endif
        dbg.obs_spd_rpm_err = s_obs_spd_pll_rpm - s_pll_omega_mech_rpm;
#endif
#endif
        }
    }
#endif
    t_post_obs = *(volatile uint32_t *)&DWT->CYCCNT;
    g_telem_dbg.obs_delta = t_post_obs - t_pre_obs;

    foc_svpwm_apply_abc(axis, uq_out, ud_out, theta_park, ia, ib, ic, id, iq);
    encoder_kick(axis->enc);
    t_post_svpwm = *(volatile uint32_t *)&DWT->CYCCNT;
    /* FOC 核心 = Clarke→PI + SVPWM/kick，不含观测器 */
    g_telem_dbg.foc_delta = (t_pre_obs - foc_t0) + (t_post_svpwm - t_post_obs);

#if M1_HFI_ENABLE
    /* T3：只提交已 harvest 的 s_harvest，不再二次 getter */
    observer_telem_publish();
#endif
    telem_bringup_tick();

    {
        volatile uint32_t isr_t1 = *(volatile uint32_t *)&DWT->CYCCNT;

        g_telem_dbg.isr_t0 = isr_t0;
        g_telem_dbg.isr_t1 = isr_t1;
        g_telem_dbg.cyccnt_end = isr_t1;
        g_telem_dbg.isr_delta = isr_t1 - isr_t0;
    }
}
