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
#include "experiment/exp_runner.h"
#include "experiment/exp_act_basic.h"
#include "motor_params_m1.h"
#include "core/motor_route.h"
#include "motor_pll.h"
#include "motor_startup.h"
#include "motor_if.h"
#include "motor_trig.h"
#include "motor_math.h"
#include "observer/emf_smo.h"
#include "observer/emf_pll.h"
#include "observer/observer_composite.h"
#include "isr/isr_state.h"
#include "isr/isr_frame.h"

/** 上电前几拍用实测角重播 PLL，吃掉编码器首帧无效造成的积分器冲顶。 */
#define M1_PLL_PRIME_TICKS   200u

/** 上一拍机械角，单位 rad。用来 unwrap（仅 getter/reset 用）。 */
static float s_pll_theta_mech_prev;
/** 1=已经有上一拍机械角。 */
static uint8_t s_pll_theta_mech_prev_valid;

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

    g_isr_m1_ctx.pole_pairs = g_m1_motor_cfg.pole_pairs;
    /* 实验开关初值 = 编译期能力宏；热路径再读 atrb，便于同次上电 A/B。 */
    g_isr_m1_ctx.atrb = 0u;
    motor_open_sweep_init(&g_isr_m1_ctx);
    /* S4a：单一选路（bringup M1_ACTIVE_ROUTE_ID → motor_route_for_boot）。 */
    motor_route_bind(&g_isr_m1_ctx, motor_route_for_boot());
    if (M1_SPEED_LOOP_ENABLE) {
        motor_outer_loop_init(&g_isr_m1_ctx);
    }
    if (M1_HFI_ENABLE) {
        g_isr_m1_ctx.mode = M1_CTRL_CURRENT_LOOP;
        g_isr_m1_ctx.id_ref = 0.0f;
        g_isr_m1_ctx.iq_ref = 0.0f;
        dbg.open_seq_phase = 0u; /* HFI IDLE */
        if (M1_EMF_PLL_ENABLE) {
            observer_bringup(&g_isr_emf_pll);
        } else {
            observer_bringup(0);
        }
        g_isr_m1_inj = observer_inj_ops();
    } else if (M1_IDENT_ENABLE || M1_DEADBAND_FLOW_ENABLE) {
        g_isr_m1_ctx.mode = M1_CTRL_CURRENT_LOOP;
        g_isr_m1_ctx.id_ref = 0.0f;
        g_isr_m1_ctx.iq_ref = 0.0f;
    } else {
        dbg.open_seq_phase = 0u;
        g_isr_m1_ctx.mode = M1_CTRL_MODE_DEFAULT;
        g_isr_m1_ctx.id_ref = 0.0f;
        if (g_isr_m1_ctx.mode == M1_CTRL_OBSERVE_ONLY ||
            g_isr_m1_ctx.mode == M1_CTRL_OPEN_LOOP) {
            motor_open_sweep_arm_v012(&g_isr_m1_ctx);
        }
    }
    g_isr_m1_ctx.ud_pi = 0.0f;
    g_isr_m1_ctx.uq_pi = 0.0f;
    g_isr_m1_ctx.ia = 0.0f;
    g_isr_m1_ctx.ib = 0.0f;
    g_isr_m1_ctx.ic = 0.0f;
    g_isr_m1_ctx.id = 0.0f;
    g_isr_m1_ctx.iq = 0.0f;
    motor_foc_loop_pi_init(&g_isr_m1_ctx);
    {
        deadband_service_boot_t db_boot;

        deadband_service_boot(&db_boot);
        dbg.deadband_nvm_loaded = db_boot.nvm_loaded;
        dbg.deadband_mode = (uint8_t)deadband_service_get_mode();
    }
    axis->motor_ctx = &g_isr_m1_ctx;

    if (M1_PLL_ENABLE) {
        motor_pll_init(&g_isr_m1_pll,
                       M1_PLL_KP,
                       M1_PLL_KI,
                       M1_PLL_OMEGA_LIMIT_RAD_S,
                       M1_PLL_INTEGRATOR_LIMIT_RAD_S);
        s_pll_theta_mech_prev_valid = 0u;
        g_isr_speed_fb_rpm = 0.0f;
    }
    if (M1_EMF_SMO_ENABLE) {
        emf_smo_init(&g_isr_emf_smo);
    }
    if (M1_EMF_PLL_ENABLE) {
        emf_pll_init(&g_isr_emf_pll);
    }
    if (axis->enc != NULL) {
        const uint16_t enc_raw0 = encoder_get_raw(axis->enc);

        g_isr_theta_mech_rad =
            M1_ENCODER_MECH_SIGN * encoder_get_angle(axis->enc, enc_raw0);
        if (M1_PLL_ENABLE) {
            motor_pll_reset(&g_isr_m1_pll, g_isr_theta_mech_rad);
            g_isr_pll_prime_ticks = 0u;
        }
    }

    if (M1_SPEED_LOOP_ENABLE) {
        g_isr_speed_slow_div = 0u;
    }

    if (M1_EXP_FRAMEWORK_ENABLE) {
        if (M1_BOOT_EXP_ID != 0u) {
            exp_runner_init();
            (void)exp_runner_select((uint8_t)M1_BOOT_EXP_ID);
            exp_runner_arm(&g_isr_m1_ctx);
        }
    }
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
    if (M1_PLL_ENABLE) {
        return g_isr_speed_fb_rpm;
    }
    return 0.0f;
}

/**
 * @brief 读取编码器机械角 PLL 转速。
 * @return 机械转速，单位 rpm。无 PLL 时返回 0。不含 OBS/HFI 速度反馈替换。
 */
float motor_current_get_enc_pll_omega_mech_rpm(void)
{
    if (M1_PLL_ENABLE) {
        return g_isr_pll_omega_mech_rpm;
    }
    return 0.0f;
}

/**
 * @brief 读取缓存的机械角。
 * @return 多圈连续机械角，单位 rad，上电相对零。来自编码器解包，不是观测角。
 */
float motor_current_get_theta_mech_rad(void)
{
    return g_isr_theta_mech_rad;
}

/**
 * @brief 读取位置环用的位置反馈角。
 * @return 多圈连续机械角，单位 rad。
 *         0=编码器解包，1=编码器 PLL，2=HFI θ̂ 解包。
 */
float motor_current_get_theta_fb_rad(void)
{
    /* M1_OUTER_THETA_FB_SRC=0：编码器解包机械角。 */
    return g_isr_theta_mech_rad;
}

/**
 * @brief 把编码器 PLL 和速度反馈清零，角度对齐当前机械角。
 * @note 给速度 ident 或模式切换前用。观测器速度 PLL 若编进来也会一起复位。
 */
void motor_current_pll_reset_now(void)
{
    if (M1_PLL_ENABLE) {
        motor_pll_reset(&g_isr_m1_pll, g_isr_theta_mech_rad);
        g_isr_pll_prime_ticks = (uint16_t)M1_PLL_PRIME_TICKS;
        g_isr_theta_mech_pll_rad = g_isr_theta_mech_rad;
        g_isr_pll_omega_mech_rpm = 0.0f;
        g_isr_speed_fb_rpm = 0.0f;
        s_pll_theta_mech_prev = g_isr_theta_mech_rad;
        s_pll_theta_mech_prev_valid = 0u;
        dbg.pll_omega_mech_rpm = 0.0f;
        dbg.pll_omega_diff_rpm = 0.0f;
        dbg.pll_omega_err_rpm = 0.0f;
        dbg.pll_theta_err_rad = 0.0f;
    }
}

void motor_current_outer_set_mode(bsp_axis_t *axis, m1_outer_mode_t mode)
{
    motor_context_t *ctx = motor_current_ctx(axis);

    if (ctx == NULL) {
        return;
    }

    motor_outer_set_mode(ctx, mode, ctx->iq,
                         motor_current_get_pll_omega_mech_rpm());
}

void motor_current_set_omega_ref_rpm(bsp_axis_t *axis, float rpm)
{
    motor_context_t *ctx = motor_current_ctx(axis);

    if (ctx == NULL) {
        return;
    }

    ctx->omega_ref = rpm;
}

void motor_current_set_theta_ref_rad(bsp_axis_t *axis, float theta_rad)
{
    motor_context_t *ctx = motor_current_ctx(axis);

    if (ctx == NULL) {
        return;
    }

    ctx->theta_ref_rad = theta_rad;
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

/**
 * @brief 重新武装开环启动序列。
 * @param axis 轴实例。找不到上下文则直接返回。
 * @note 仅 STARTUP 编进来时有效；会清电流 PI。未开启动时为空操作。
 */
void motor_startup_arm_axis(bsp_axis_t *axis)
{
    (void)axis;
}

/**
 * @brief 电流环一步：转交 isr_entry（20 kHz 编排）。
 * @param axis 轴实例。不可为 NULL，且必须已绑编码器和 PWM。
 */
void motor_current_tick(bsp_axis_t *axis)
{
    isr_entry_tick(axis);
}


uint8_t motor_current_outer_run_enable(void)
{
    return g_isr_outer_run_enable;
}

void motor_current_outer_tick_2k(void)
{
    g_isr_diag_tim6_cnt++;
    if (g_isr_outer_run_enable == 0u) {
        return;
    }
    motor_outer_loop_tick(&g_isr_m1_ctx);
}

uint32_t motor_current_get_tim6_cnt(void)
{
    return g_isr_diag_tim6_cnt;
}

uint32_t motor_current_get_foc_cnt(void)
{
    return g_isr_diag_foc_cnt;
}
