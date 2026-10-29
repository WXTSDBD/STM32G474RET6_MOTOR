/**
 * @file motor_current.c
 * @date 2026-10-06
 * @brief M1 电流环节拍编排：采样、选 Park 角、电流 PI、SVPWM。
 *
 * 本文件负责把一步串起来。已绑定 route 时，Park 和速度反馈只解引用角度指针。
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
#include "motor_current_exp_hooks.h"
#include "motor_foc_loop.h"
#include "motor_open_sweep.h"
#include "motor_outer_loop.h"
#include "motor_phase_binding.h"
#include "motor_cfg.h"
#if M1_EXP_FRAMEWORK_ENABLE
#include "experiment/exp_runner.h"
#include "experiment/exp_act_basic.h"
#endif

#if M1_EXP_FRAMEWORK_ENABLE
/* 1 = experiment table owns the drive (armed): HFI script must not write refs. */
static uint8_t exp_owns_drive(void)
{
    return (exp_runner_is_armed() != 0u) ? 1u : 0u;
}
#endif
#include "motor_params_m1.h"
#include "core/motor_route.h"

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
#if M1_EMF_SMO_ENABLE
#include "observer/emf_smo.h"
#endif
#if M1_EMF_PLL_ENABLE
#include "observer/emf_pll.h"
#endif
#if M1_HFI_ENABLE
#include "observer/observer_composite.h"
#endif

/** M1 控制上下文。电流环和外环都写这里。 */
static motor_context_t s_m1_ctx;

#if M1_EMF_SMO_ENABLE
static emf_smo_t s_emf_smo;
#endif
#if M1_EMF_PLL_ENABLE
static emf_pll_t s_emf_pll;
#endif
#if M1_PLL_ENABLE
/** 外环实际速度反馈，单位 rpm。有感走编码器 PLL；无感速度反馈打开时走观测 PLL。 */
static float s_speed_fb_rpm;
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
/**
 * 外环使能（20 kHz 拍按运行模式刷新，供 TIM6 2 kHz ISR 读取）。
 * 语义与旧的局部变量 run_speed_loop 完全一致：I/F 未交接、只切角、滑行段 ⇒ 0。
 * 单字节写，Cortex-M4 上是原子的，无需临界区。
 */
static volatile uint8_t s_outer_run_enable = 1u;
/** [台架诊断] TIM6 ISR 次数 / FOC ISR 次数 —— 一次录波即可反推两者真实频率比。 */
static volatile uint32_t s_diag_tim6_cnt;
static volatile uint32_t s_diag_foc_cnt;
#endif
#if M1_HFI_ENABLE && M1_HFI_MOTION_BYPASS_ENABLE
/** RUN 起用上一拍 θ̂ 做扇区重构，单位 rad。 */
static float s_hfi_recon_theta;
/** 1=重构角有效。未进 RUN 前不要用编码器角改电流。 */
static uint8_t s_hfi_recon_theta_ok;
#endif
#if M1_HFI_ENABLE
/** 本轴观测器 ops。init 后冻结。 */
static const observer_ops_t *s_m1_obs;
/** 本轴注入表。 */
static const observer_inj_ops_t *s_m1_inj;
#endif


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
    motor_open_sweep_init(&s_m1_ctx);
    /* S4a：单一选路（bringup M1_ACTIVE_ROUTE_ID → motor_route_for_boot）。 */
    motor_route_bind(&s_m1_ctx, motor_route_for_boot());
#if M1_SPEED_LOOP_ENABLE && (!M1_IF_ENABLE || M1_IF_TO_OBS_ENABLE)
    motor_outer_loop_init(&s_m1_ctx);
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
#else
    dbg.open_seq_phase = 0u;
    s_m1_ctx.mode = M1_CTRL_MODE_DEFAULT;
    s_m1_ctx.id_ref = 0.0f;
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
#if M1_EMF_SMO_ENABLE
    emf_smo_init(&s_emf_smo);
#endif
#if M1_EMF_PLL_ENABLE
    emf_pll_init(&s_emf_pll);
#endif
#if M1_PLL_ENABLE
    s_speed_fb_rpm = 0.0f;
#endif
    if (axis->enc != NULL) {
        const uint16_t enc_raw0 = encoder_get_raw(axis->enc);

        s_theta_mech_rad =
            M1_ENCODER_MECH_SIGN * encoder_get_angle(axis->enc, enc_raw0);
#if M1_PLL_ENABLE
        motor_pll_reset(&s_m1_pll, s_theta_mech_rad);
        s_pll_prime_ticks = 0u;
#endif
    }

#if M1_SPEED_LOOP_ENABLE && (!M1_IF_ENABLE || M1_IF_TO_OBS_ENABLE)
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
    dbg.pll_omega_mech_rpm = 0.0f;
    dbg.pll_omega_diff_rpm = 0.0f;
    dbg.pll_omega_err_rpm = 0.0f;
    dbg.pll_theta_err_rad = 0.0f;
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

#if M1_SPEED_LOOP_ENABLE
    s_diag_foc_cnt++;   /* [台架诊断] 与 TIM6 计数配对算频率比 */
#endif

    ua_hfi = 0.0f;
    ub_hfi = 0.0f;

    if (axis->adc.cal_active) {
        return;
    }

#if M1_ENC_OPTIONAL_ENABLE
    /* 141 纯无感：ISR 不读 SPI、不跑 enc PLL；踢极性无 Δθ 时默认北。 */
    enc_raw = 0u;
    dbg.enc_raw = 0.0f;
    s_theta_mech_rad = 0.0f;
    theta = 0.0f;
#else
    enc_raw = encoder_get_raw(axis->enc);
    /* OPT12 ch1=enc_raw，须写入；非 OPT12 也保留镜像。 */
    dbg.enc_raw = (float)enc_raw;
    /* MECH_SIGN 只翻速度/位置反馈；电角仍用芯片 raw + 零偏。 */
    s_theta_mech_rad =
        M1_ENCODER_MECH_SIGN * encoder_get_angle(axis->enc, enc_raw);
    theta = encoder_get_theta_el(axis->enc, enc_raw, ctx->pole_pairs,
                                 encoder_get_theta_el_offset(axis->enc));
#endif
#if M1_PLL_ENABLE
    {
#if !M1_ENC_OPTIONAL_ENABLE
        const float theta_mech = s_theta_mech_rad;

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
#else
        s_pll_omega_mech_rpm = 0.0f;
        s_theta_mech_pll_rad = 0.0f;
        s_speed_fb_rpm = 0.0f;
        dbg.pll_omega_mech_rpm = 0.0f;
#endif
#if M1_HFI_ENABLE && M1_HFI_SPEED_FB_ENABLE && M1_HFI_MOTION_BYPASS_ENABLE
        /* 速度反馈跟当前角度源。无感源给观测转速，有感源给编码器 PLL。 */
        if (motor_angle_active() != NULL) {
            s_speed_fb_rpm = motor_angle_active()->get_omega_rpm(&s_m1_ctx);
        } else if (observer_sensed_fb_active() == 0u) {
            const float rpm_scale =
                60.0f / (2.0f * 3.14159265f * (float)s_m1_ctx.pole_pairs);

            if (observer_pub_ss() >= 1.0f) {
                s_speed_fb_rpm = observer_pub_smo_rpm();
            } else {
                s_speed_fb_rpm = observer_get_pll_int_el() * rpm_scale;
            }
        }
#endif
#if !M1_ENC_OPTIONAL_ENABLE
#else
#endif
    }
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

#if M1_SPEED_IDENT_ENABLE
    if (s_speed_ident_booted == 0u) {
        /* 。Bode 同：PWM/ADC/编码。SPI 已跑后再 arm，并。PLL 积分 */
        motor_current_pll_reset_now();
        deadband_flow_boot(ctx);
        speed_ident_flow_init(ctx);
        s_speed_ident_booted = 1u;
    }
    deadband_flow_tick(ctx);
#else
    /* 辨识/死区：OFF 宏时空桩，见 motor_current_exp_hooks.c */
    motor_current_exp_flow_tick(ctx);
#endif

#if M1_SPEED_LOOP_ENABLE
    /* 注意：s_if_to_obs_handed 是运行时变量，不能写。#if（预处理当成 0。
     * 会把 I/F→OBS 的速度环整段编译掉 。iq_ref 钉死在交接值，Uq 顶满）。*/
    {
        uint8_t run_speed_loop = 1u;

        /* 把本拍的运行条件交给 TIM6 2 kHz ISR（单字节写，原子） */
        s_outer_run_enable = run_speed_loop;
#if M1_OUTER_IN_TIM6
        /*
         * 外环已移到 TIM6 2 kHz 中断（见 motor_current_outer_tick_2k）。
         * 这里只保留 M1_MIT_TORQUE_LAW_20KHZ 的 20 kHz 补刷（它本来就是 20 kHz 量）。
         */
#else
        if (run_speed_loop != 0u) {
            if (++s_speed_slow_div >= M1_SPEED_DECIM) {
                s_speed_slow_div = 0u;
                motor_outer_loop_tick(ctx);
            }
        }
#endif
    }
#endif

    if (ctx->mode == M1_CTRL_CURRENT_LOOP) {
#if M1_IF_ENABLE
        {
            motor_if_step_t if_step = motor_if_tick(ctx, theta_enc_park);

            if (motor_if_is_driving() != 0u) {
                theta_park = if_step.theta_park;
                ctx->id_ref = if_step.id_ref;
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
            }
            dbg.startup_state = (uint8_t)if_step.state;
            dbg.startup_omega_mech_rpm = if_step.omega_meas_rpm;
            dbg.if_omega_cmd_rpm = if_step.omega_cmd_rpm;
            dbg.if_theta_err_rad = if_step.theta_err_rad;
        }
#else
        startup = motor_startup_tick(ctx, theta_enc_park);
#if M1_IDENT_ENABLE
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
    }
#if M1_HFI_ENABLE
    /*
     * 有感对照：只推时钟，Park=enc（SENSED_PAIRS 段）。
     * 无感：始终 pre_park（含 inj_off），与 6059/141 一致；Park=发布角。
     * 交接省 ISR 的 hand_keep 先收回，避免无感交接角路径与冻结点不一致。
     *
     * 标定链锁轴：上方已写死 theta_park=target；此处若再走观测角会把
     * FIX_THETA/ALIGN 整段盖掉（Rs 灌 Id 却跟转子转、无锁轴力矩）。
     */
    if (deadband_flow_id_cal_active() == 0u) {
        if (motor_angle_active() != NULL) {
            const angle_ops_t *ang = motor_angle_active();

            motor_angle_note_enc(theta_enc_park);
            if (ang->update != NULL) {
                ang->update(&s_m1_ctx, M1_CTRL_TS_S);
            }
            theta_park = ang->get_theta(&s_m1_ctx);
            if (ang->post_park != NULL) {
#if M1_HFI_MOTION_BYPASS_ENABLE
                if (observer_get_stage() == OBS_STAGE_RUN) {
                    s_hfi_recon_theta = theta_park;
                    s_hfi_recon_theta_ok = 1u;
                }
#endif
#if M1_HFI_QKICK_AFTER_LOCK_ENABLE
                if (observer_consume_pi_reset() != 0u) {
                    foc_pi_reset(&ctx->pi_iq);
                    ctx->uq_pi = 0.0f;
                }
#endif
            }
        } else if (observer_sensed_fb_active() != 0u) {
            observer_sensed_keep(theta_enc_park, M1_CTRL_TS_S);
            theta_park = theta_enc_park;
        } else {
            s_m1_obs->pre_park(theta_enc_park, M1_CTRL_TS_S);
            theta_park = s_m1_obs->get_theta();
#if M1_HFI_MOTION_BYPASS_ENABLE
            if (observer_get_stage() == OBS_STAGE_RUN) {
                s_hfi_recon_theta = theta_park;
                s_hfi_recon_theta_ok = 1u;
            }
#endif
#if M1_HFI_QKICK_AFTER_LOCK_ENABLE
            if (observer_consume_pi_reset() != 0u) {
                /* 踢时只清 Iq PI，Id 积分保持 */
                foc_pi_reset(&ctx->pi_iq);
                ctx->uq_pi = 0.0f;
            }
#endif
        }
    }
    /*
     * 进有感段（REV141 SENSED_PAIRS）第一拍：电流环 PI 与 Ud/Uq 全部清零。
     * 不清会把无感/刹停段残留的积分带进有感段 —— 实测有感段 |U| 比无感大一倍、
     * d 轴恒顶约 -5 V、δ 偏 37°。
     */
    if (observer_consume_sens_pi_clear() != 0u) {
        foc_pi_reset(&ctx->pi_id);
        foc_pi_reset(&ctx->pi_iq);
        ctx->ud_pi = 0.0f;
        ctx->uq_pi = 0.0f;
    }
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

            if (exp_owns_drive() == 0u) {
                ctx->omega_ref = rpm_cmd;
            }
            if (ctx->outer_mode != M1_OUTER_SPEED) {
                /* ω* 先写 0 再切速度模式，避免 bumpless 把积分预成 -Kp·目标。 */
                ctx->omega_ref = 0.0f;
                motor_outer_set_mode(ctx, M1_OUTER_SPEED, 0.0f, 0.0f);
                if (exp_owns_drive() == 0u) {
                    ctx->omega_ref = rpm_cmd;
                }
            }
        } else {
            /* 速度环关：由 QKICK/HFI 写 id_ref/iq_ref/omega_ref */
            if (exp_owns_drive() == 0u) {
                ctx->id_ref = observer_get_id_ref();
                ctx->iq_ref = observer_get_iq_ref();
                ctx->omega_ref = observer_get_speed_ref_rpm();
            }
            if (ctx->outer_mode != M1_OUTER_DISABLED) {
                motor_outer_set_mode(ctx, M1_OUTER_DISABLED, 0.0f, 0.0f);
            }
        }
        /* 段表在写转速。前馈保持 0，编码器转速和指令转速不进 HFI。 */
        observer_set_omega_ff_el(0.0f);
#endif
    }
#endif
#endif
    dbg.foc_theta_el = theta_park;

#if M1_EXP_FRAMEWORK_ENABLE
    /* pilot: table-driven UD_CAL (EXP_ACT_THETA_REF) overrides Park angle. */
    if (exp_runner_is_armed() == 0u) {
        exp_act_theta_clear();
    } else if ((exp_runner_active_id() == EXP_ID_UD_CAL) &&
               (exp_act_theta_is_active() != 0u)) {
        theta_park = exp_act_theta_get_rad();
        dbg.foc_theta_el = theta_park;
    }
    else if ((motor_angle_active() == NULL) &&
             (exp_runner_active_id() == EXP_ID_REV141) &&
             (exp_act_angle_src_is_active() != 0u) &&
             (exp_act_angle_src() != 0u)) {
        theta_park = theta_enc_park;
        dbg.foc_theta_el = theta_park;
    }
#endif
    motor_trig_sincos(theta_park, &cos_el, &sin_el);
    Park_Transform_sc(i_alpha, i_beta, sin_el, cos_el, &id, &iq);
    ctx->id = id;
    ctx->iq = iq;

#if M1_HFI_ENABLE
    /* 有感对照不解调。无感始终 post_park；inj_off 时内部 hat_hold 走 coast。
     * 标定链（Rs/L）锁轴：不跑 HFI 解调，避免与 l_hfi 双极注入抢状态。 */
    if (deadband_flow_id_cal_active() == 0u) {
        if (motor_angle_active() != NULL) {
            const angle_ops_t *ang = motor_angle_active();

            if (ang->post_park != NULL) {
                ang->post_park(&s_m1_ctx, id, iq, i_alpha, i_beta);
            }
        } else if (observer_sensed_fb_active() == 0u) {
            s_m1_obs->post_park(id, iq, i_alpha, i_beta);
        }
    }
#if M1_HFI_IQ_AUTH_ENABLE
    /*
     * 注入仍开时扫权威限幅。交接后 inj_off：天花板已钉 HI（=速度环满幅），
     * 解调也不再推 auth，PI 限幅保持上一拍即可。
     */
    if (observer_hfi_inj_off() == 0u) {
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
    }
#endif
#endif

#if M1_HFI_ENABLE
    /*
     * 无扰：id* = (1-soft)·id → 误差 = -soft·id；
     * soft=0 时误差为 0；soft→1 → id*→0。须在 foc_loop 之前。
     * 标定链（Pass0/Rs/L）与 HFI 互斥：表在跑就别改 id_ref。
     */
    if ((deadband_flow_id_cal_active() == 0u) &&
        (observer_id_pi_bypass() == 0u)) {
        const float soft = observer_id_pi_soft_scale();

        if (soft < 1.0f) {
            ctx->id_ref = (1.0f - soft) * id;
        } else {
            ctx->id_ref = 0.0f;
        }
    }
#endif

    motor_foc_loop_dbg_id_ref(ctx);

    if (ctx->mode == M1_CTRL_CURRENT_LOOP) {
        float id_for_pi = id;
#if !M1_IF_ENABLE
        motor_startup_finish_tick(ctx, iq, &startup);
#endif
#if M1_EXP_FRAMEWORK_ENABLE
        if ((exp_runner_is_armed() != 0u) &&
            (exp_runner_active_id() == EXP_ID_UD_CAL) &&
            (exp_act_theta_is_active() != 0u)) {
            ctx->id_ref = exp_act_theta_get_id_a();
            ctx->iq_ref = 0.0f;
            /* current PI only runs in CURRENT_LOOP; force it during the experiment */
            ctx->mode = M1_CTRL_CURRENT_LOOP;
            startup.iq_ref = 0.0f;
            startup.use_fixed_uq = 0u;
        }
#endif
#if M1_EXP_FRAMEWORK_ENABLE
        if ((exp_runner_is_armed() != 0u) &&
            (exp_runner_active_id() == EXP_ID_REV141)) {
            ctx->mode = M1_CTRL_CURRENT_LOOP;
        }
#endif
        motor_foc_loop_tick(ctx, id_for_pi, iq, &startup, theta_enc_park);
    }

    if (ctx->mode == M1_CTRL_OBSERVE_ONLY || ctx->mode == M1_CTRL_OPEN_LOOP) {
        motor_open_sweep_tick(ctx);
    }

    switch (ctx->mode) {
    case M1_CTRL_CURRENT_LOOP:
        if (motor_current_exp_try_ld_lq_udq(ctx, id, iq, &ud_out, &uq_out) !=
            0u) {
            break;
        }
        if (startup.use_fixed_uq) {
            ud_out = 0.0f;
            uq_out = startup.uq_out;
        } else {
            ud_out = ctx->ud_pi + deadband_service_ud_inject(id);
            uq_out = ctx->uq_pi;
#if M1_HFI_ENABLE
            /* 标定链：HFI 不掺 Ud；其余旁路/soft 与 141 无感一致。 */
            if (deadband_flow_id_cal_active() != 0u) {
                dbg.foc_ud_pi = ctx->ud_pi;
            } else if (observer_id_pi_bypass() != 0u) {
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
        /*
         * 标定链（Rs/L/Pass0）与 HFI 互斥：IDLE 的 override(0,0) 会盖住 Rs 斜坡
         * 并每拍清 PI（表现为前几秒无电流、随后 id_ref 已爬高而突跳）。
         */
        if (deadband_flow_id_cal_active() == 0u) {
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
            } else if ((s_m1_inj != 0) &&
                       (observer_hfi_inj_off() == 0u) &&
                       (((motor_angle_active() != NULL) &&
                         (motor_angle_sensorless() != 0u)) ||
                        ((motor_angle_active() == NULL) &&
                         (observer_sensed_fb_active() == 0u)))) {
                /* 有感不叠注入。交接后幅值为 0 也不叠。 */
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
        if (motor_current_exp_try_open_udq(ctx, &ud_out, &uq_out) != 0u) {
            break;
        }
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
    /* 瘦路径：只 harvest + OPT12 需要的 hfi_stage（勿再满拷 read_view） */
    observer_telem_harvest();
    dbg.hfi_stage =
        (float)observer_get_stage() + observer_get_eps_dead();
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
#endif
#endif

    t_pre_obs = *(volatile uint32_t *)&DWT->CYCCNT;
#if M1_EMF_VEQ_ENABLE || M1_EMF_SMO_ENABLE || M1_EMF_PLL_ENABLE
    /* REV141 有感对照：跳过 SMO/EMF-PLL/pub；阶梯仍由 HFI on_angle + ω* 推进。 */
#if M1_HFI_ENABLE
    if (((motor_angle_active() != NULL) && (motor_angle_sensorless() == 0u)) ||
        ((motor_angle_active() == NULL) && (observer_sensed_fb_active() != 0u))) {
        /* 有感段：观测器热路径空转。 */
    } else
#endif
    {
        float omega_mech_rpm = 0.0f;
        float u_alpha;
        float u_beta;
        float theta_obs_ref = theta_enc_park;
        uint8_t smo_hold = 0u;

#if M1_PLL_ENABLE
        omega_mech_rpm = s_speed_fb_rpm;
#endif
#if M1_HFI_ENABLE && M1_HFI_MOTION_BYPASS_ENABLE && M1_EMF_PLL_ENABLE && \
    (M1_EMF_SMO_ENABLE || M1_EMF_VEQ_ENABLE)
        /* 旁路相对 θ̂_hfi。速度环未接管前每个周期清掉，避免零速假转速和编码器初值。*/
        theta_obs_ref = observer_get_theta_hat();
        /*
         * 极性 flip：HFI 侧已 θ̂+=π。此处只消费标志，不清 SMO/VEQ/EMF，
         * 否则踢后观测从零重建，到 PUB_UP 过嫩（2251 交接毛刺）。
         */
        (void)observer_take_polarity_flip();
#endif
        if (smo_hold != 0u) {
            /* 本拍不更 SMO/PLL（极性翻面或低速互斥）。 */
        } else {
        /* 与电流环同思路：反 Park 复用本拍 Park 。sin/cos（θ_park 帧下。ud/uq。*/
        u_alpha = ud_out * cos_el - uq_out * sin_el;
        u_beta = ud_out * sin_el + uq_out * cos_el;
        /* 桥上仍叠 Vh；磁链/SMO 只吃基波 */
        u_alpha -= ua_hfi;
        u_beta -= ub_hfi;

#if M1_EMF_SMO_ENABLE
        {
            (void)emf_smo_lpf_sched_update(omega_mech_rpm);

            emf_smo_update(&s_emf_smo, i_alpha, i_beta, u_alpha, u_beta,
                           theta_obs_ref, omega_mech_rpm);
        }
#endif
#if M1_EMF_PLL_ENABLE
#if M1_EMF_PLL_USE_SMO
        observer_emf_update(s_emf_smo.e_alpha, s_emf_smo.e_beta,
                            theta_obs_ref, M1_CTRL_TS_S);
#else
        observer_emf_update(s_emf_veq.e_alpha, s_emf_veq.e_beta,
                            theta_obs_ref, M1_CTRL_TS_S);
#endif
#if M1_HFI_ENABLE && M1_HFI_MOTION_BYPASS_ENABLE && M1_EMF_PLL_ENABLE
        {
            const float emf_th = observer_emf_theta_hat();
            const float emf_w = observer_emf_omega_el();
            float dth = emf_th - theta_obs_ref;

            while (dth > 3.14159265f) {
                dth -= 6.2831853f;
            }
            while (dth < -3.14159265f) {
                dth += 6.2831853f;
            }
            /* 发布门槛用 PLL ω；400 点 MA 只给观测镜像，OPT12 不跑。 */
            observer_pub_step(emf_th, emf_w, dth);
        }
#endif
#endif
        }
    }
#endif
    t_post_obs = *(volatile uint32_t *)&DWT->CYCCNT;
    g_telem_dbg.obs_delta = t_post_obs - t_pre_obs;

    foc_svpwm_apply_abc_sc(axis, uq_out, ud_out, theta_park, sin_el, cos_el,
                           ia, ib, ic, id, iq);
#if !M1_ENC_OPTIONAL_ENABLE
    encoder_kick(axis->enc);
#endif
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

#if M1_SPEED_LOOP_ENABLE
/**
 * @brief 2 kHz 外环（TIM6 中断里调用，与 20 kHz FOC 内环剥离）。
 *
 * 调用上下文：TIM6_DAC_IRQHandler，优先级 3（低于 FOC 的 ADC 优先级 2 ⇒ FOC 可抢占）。
 * 只允许从 TIM6 中断调用：内部直接操作 s_m1_ctx 的外环状态。
 *
 * 与 20 kHz 内环的共享面：ctx->iq_ref / id_ref / omega_ref 三个 32 位 float（单次写，原子）；
 * 外环 PI 状态、斜坡状态只被本函数触碰，无需临界区。
 * 若 FOC 在本函数中途抢占并改了 ctx->outer_mode，本拍剩余计算基于旧模式，下一拍即恢复一致，
 * 且 DISABLED 模式在 motor_outer_loop_tick 的 switch 里无分支 ⇒ 不会误写 iq_ref。
 */
uint8_t motor_current_outer_run_enable(void)
{
    return s_outer_run_enable;
}

void motor_current_outer_tick_2k(void)
{
    s_diag_tim6_cnt++;
    if (s_outer_run_enable == 0u) {
        return;
    }
    motor_outer_loop_tick(&s_m1_ctx);
}

/** @brief [台架诊断] TIM6 ISR 累计次数。 */
uint32_t motor_current_get_tim6_cnt(void)
{
    return s_diag_tim6_cnt;
}

/** @brief [台架诊断] FOC ISR 累计次数。 */
uint32_t motor_current_get_foc_cnt(void)
{
    return s_diag_foc_cnt;
}
#endif
