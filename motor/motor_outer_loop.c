/**
 * @file motor_outer_loop.c
 * @date 2026-10-06
 * @brief 外环实现：力矩、速度、位置，以及无扰切换。
 *
 * 本文件写出 iq_ref。电流 PI 和 Park 不在这里。
 * 节拍限制见 motor_outer_loop.h 文件头。
 * MIT 并行力矩律（M1_MIT_TORQUE_LAW）可在电流环节拍刷新 iq_ref。
 * SPEED/TORQUE 另挂 g_outer_speed / g_outer_torque；热路径优先走表。
 *
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
 */

#include "motor_outer_loop.h"

#include <math.h>
#include <stddef.h>

#include "dbg_monitor.h"
#include "exp_mark.h"
#include "deadband_flow.h"
#include "foc_pi.h"
#include "motor_current.h"
#include "motor_if.h"
#include "motor_params_m1.h"
#include "core/motor_route.h"
#include "speed_ident_flow.h"
#include "experiment/exp_runner.h"

static float s_omega_ramped_rpm;
/* 1=减速/反向保持用 DECEL 档，直到 ω* 到达目标（过零后不改 ACCEL） */
static uint8_t s_omega_ramp_decel_hold;
static float s_iq_ref_slew;

/**
 * @brief 驱动=+|Idrv|、regen=−|Ireg| → out_min/max（与 ω 转向无关）
 * @note I/F/Park 本工程：正反转拖动电流在交接瞬间均为 +Iq（2121）。
 *       DIR 只乘 ω 目标；勿把限权做成 DIR·Iq，否则反转浅刹会夹死 +Iq。
 */
static void outer_iq_lims_from_regen_drive(float regen_abs,
                                            float drive_abs,
                                            float *out_min,
                                            float *out_max)
{
    *out_min = -regen_abs;
    *out_max = drive_abs;
}

static void outer_apply_regen_drive_limits(motor_context_t *ctx,
                                            float regen_abs,
                                            float drive_abs)
{
    float lo;
    float hi;

    if (ctx == NULL) {
        return;
    }
    outer_iq_lims_from_regen_drive(regen_abs, drive_abs, &lo, &hi);
    ctx->pi_speed.out_min = lo;
    ctx->pi_speed.out_max = hi;
    ctx->pi_speed.int_min = lo;
    ctx->pi_speed.int_max = hi;
    if (ctx->pi_speed.integrator > hi) {
        ctx->pi_speed.integrator = hi;
    } else if (ctx->pi_speed.integrator < lo) {
        ctx->pi_speed.integrator = lo;
    }
}

static void outer_soft_brake_snap_iq(motor_context_t *ctx)
{
    float lo;
    float hi;

    if (ctx == NULL) {
        return;
    }
    lo = ctx->pi_speed.out_min;
    hi = ctx->pi_speed.out_max;
    if ((ctx->iq_ref > hi) || (ctx->iq_ref < lo)) {
        ctx->iq_ref = M1_IF_OBS_SPEED_IQ_BOOT_A;
        if (ctx->iq_ref > hi) {
            ctx->iq_ref = hi;
        } else if (ctx->iq_ref < lo) {
            ctx->iq_ref = lo;
        }
    }
}

static float outer_iq_ref_min(const motor_context_t *ctx)
{
    if (ctx != NULL) {
        return ctx->pi_speed.out_min;
    }
        return M1_SPEED_PI_OUT_MIN;
}

static float outer_clamp_iq_ref(const motor_context_t *ctx, float ref)
{
    const float iq_min = outer_iq_ref_min(ctx);
    const float iq_max = (ctx != NULL) ? ctx->pi_speed.out_max : M1_SPEED_PI_OUT_MAX;

    if (ref > iq_max) {
        return iq_max;
    }
    if (ref < iq_min) {
        return iq_min;
    }
    return ref;
}


/**
 * @brief ω* 匀变速斜坡：减速/反向全程同一 DECEL |dn/dt|，同向加速才用 ACCEL。
 * @note 旧逻辑异号过零后改 ACCEL，反向变成「急刹到 0 + 慢爬升」；DECEL_BRAKE 看 fb≈ω*
 *       （无感）时还可能钉不住制动。hold 位同时给制动地板用。
 */
static float outer_omega_ref_ramped(motor_context_t *ctx)
{
    const float target = ctx->omega_ref;
    const motor_route_t *route = motor_route_active();
    float ramp_up = M1_SPEED_OMEGA_RAMP_RPM_S;
    float ramp_dn = ramp_up;
    float step_up;
    float step_dn;
    const float cur = s_omega_ramped_rpm;
    float delta;
    float step;
    const float eps_rpm = 1.0f;

    /* 斜坡只认 route->gains；无 route 时 accel=decel=宏默认。 */
    if ((route != NULL) && (route->gains != NULL)) {
        ramp_up = route->gains->omega_ramp_rpm_s;
        ramp_dn = route->gains->omega_ramp_decel_rpm_s;
    }
    step_up = ramp_up * M1_SPEED_TS_S;
    step_dn = ramp_dn * M1_SPEED_TS_S;
    delta = target - cur;

    /* 收 |n| 或异号 → 进入减速保持，直到到达目标。 */
    if ((fabsf(target) + eps_rpm < fabsf(cur)) || ((cur * target) < 0.0f)) {
        s_omega_ramp_decel_hold = 1u;
    }
    if (fabsf(delta) <= eps_rpm) {
        s_omega_ramped_rpm = target;
        s_omega_ramp_decel_hold = 0u;
        return s_omega_ramped_rpm;
    }

    step = (s_omega_ramp_decel_hold != 0u) ? step_dn : step_up;

    if (delta > step) {
        s_omega_ramped_rpm = cur + step;
    } else if (delta < -step) {
        s_omega_ramped_rpm = cur - step;
    } else {
        s_omega_ramped_rpm = target;
        s_omega_ramp_decel_hold = 0u;
    }
    return s_omega_ramped_rpm;
}

static float outer_speed_pi_step(motor_context_t *ctx,
                                  float omega_ref,
                                  float omega_fb,
                                  uint8_t hold_i)
{
    float fb = omega_fb;
    float ref_pi = omega_ref;
    float fb_pi;
    fb_pi = fb;
    float iq = foc_pi_step_beta_hold_i(&ctx->pi_speed,
                                        ref_pi,
                                        fb_pi,
                                        M1_SPEED_PI_BETA,
                                        hold_i);
    if (M1_SPEED_IQ_SLEW_ENABLE) {
        const float step = M1_SPEED_IQ_SLEW_A_PER_S * M1_SPEED_TS_S;
        float delta = iq - s_iq_ref_slew;

        if (delta > step) {
            s_iq_ref_slew += step;
        } else if (delta < -step) {
            s_iq_ref_slew -= step;
        } else {
            s_iq_ref_slew = iq;
        }
        return outer_clamp_iq_ref(ctx, s_iq_ref_slew);
    }
    return outer_clamp_iq_ref(ctx, iq);
}

static float outer_clamp_omega_ref(float rpm)
{
    if (rpm > M1_POS_OMEGA_MAX_RPM) {
        return M1_POS_OMEGA_MAX_RPM;
    }
    if (rpm < -M1_POS_OMEGA_MAX_RPM) {
        return -M1_POS_OMEGA_MAX_RPM;
    }
    return rpm;
}

static uint8_t s_pos_err_hold_zone;

static float outer_pos_fabsf(float x)
{
    return (x < 0.0f) ? -x : x;
}

static void outer_pos_err_hyst_reset(void)
{
    s_pos_err_hold_zone = 0u;
}

/** 滞环：|err|<ENTER 进区 → ω_ref=0；在区内 |err|>EXIT 才重新给 P 输出 */
static float outer_pos_omega_from_err(float err_rad)
{
    const float aerr = outer_pos_fabsf(err_rad);

    if (s_pos_err_hold_zone) {
        if (aerr > M1_POS_ERR_HYST_EXIT_RAD) {
            s_pos_err_hold_zone = 0u;
        }
    } else if (aerr < M1_POS_ERR_HYST_ENTER_RAD) {
        s_pos_err_hold_zone = 1u;
    }

    if (s_pos_err_hold_zone) {
        return 0.0f;
    }
    return outer_clamp_omega_ref(M1_POS_KP_RPM_PER_RAD * err_rad);
}

static uint8_t s_pos_p_div;

static void outer_pos_p_decim_arm(void)
{
    s_pos_p_div = M1_POS_DECIM;
}

/**
 * @brief 装速度 PI，默认 DISABLED。
 * @param ctx 控制上下文。不可为 NULL。
 */
void motor_outer_loop_init(motor_context_t *ctx)
{
    float kp = M1_SPEED_PI_KP;
    float ki = M1_SPEED_PI_KI;
    float out_max = M1_SPEED_PI_OUT_MAX;
    float out_min = M1_SPEED_PI_OUT_MIN;
    float int_max = M1_SPEED_PI_INT_MAX;
    float int_min = M1_SPEED_PI_INT_MIN;

    if (ctx == NULL) {
        return;
    }

    /* S4a：有 route gains 时速度环吃表（i_lim→输出限幅）。 */
    if (ctx->gains != NULL) {
        const motor_gains_t *g = ctx->gains;

        kp = g->spd.kp;
        ki = g->spd.ki;
        if (g->spd.i_lim_a > 0.0f) {
            out_max = g->spd.i_lim_a;
            out_min = -g->spd.i_lim_a;
            int_max = out_max;
            int_min = out_min;
        }
    }

    foc_pi_init(&ctx->pi_speed, kp, ki, out_min, out_max, int_min, int_max);

    ctx->outer_mode = M1_OUTER_DISABLED;
    ctx->omega_ref  = 0.0f;
    ctx->theta_ref_rad = 0.0f;
    ctx->iq_cmd     = 0.0f;
    ctx->iq_ff      = 0.0f;
    ctx->iq_fric_ff = 0.0f;
    ctx->traj_vel_rpm = 0.0f;
    if (M1_POS_LOOP_ENABLE && M1_POS_ERR_HYST_ENABLE) {
        outer_pos_err_hyst_reset();
    }
    if (M1_POS_LOOP_ENABLE && (M1_POS_DECIM > 1u)) {
        outer_pos_p_decim_arm();
    }
    if (M1_SPEED_OMEGA_RAMP_ENABLE) {
        s_omega_ramped_rpm = 0.0f;
        s_omega_ramp_decel_hold = 0u;
    }
    if (M1_SPEED_IQ_SLEW_ENABLE) {
        s_iq_ref_slew = 0.0f;
    }
}


/**
 * @brief 力矩外环：iq_ref = clamp(iq_cmd)。
 * @param ctx 控制上下文。
 * @param theta_fb_rad 未用。
 * @param omega_rpm 未用。
 * @return iq_ref，单位 A。
 */
float outer_ops_torque_tick(motor_context_t *ctx,
                            float theta_fb_rad,
                            float omega_rpm)
{
    (void)theta_fb_rad;
    (void)omega_rpm;
    return outer_clamp_iq_ref(ctx, ctx->iq_cmd);
}

/**
 * @brief 速度外环：ident / 电压饱和冻积分 / 斜坡 + 速度 PI。
 * @param ctx 控制上下文。
 * @param theta_fb_rad 未用（位置环才吃角）。
 * @param omega_rpm 机械转速反馈，单位 rpm。
 * @return iq_ref，单位 A。
 */
float outer_ops_speed_tick(motor_context_t *ctx,
                           float theta_fb_rad,
                           float omega_rpm)
{
    uint8_t speed_hold_i = 0u;
    float iq;

    (void)theta_fb_rad;

    /* 电压饱和：冻积分，避免 iq_ref 顶满而电流环饱和 */
    {
        const float ud = dbg.foc_ud_out;
        const float uq = dbg.foc_uq_out;
        const float um2 = ud * ud + uq * uq;
        const float lim = 0.95f * M1_PI_V_MAX;

        if (um2 >= (lim * lim)) {
            speed_hold_i = 1u;
        }
    }
    /* 现网配方：ω 斜坡常开，直接走 ramped 路径，不按宏分岔。 */
    iq = outer_speed_pi_step(ctx,
                             outer_omega_ref_ramped(ctx),
                             omega_rpm,
                             speed_hold_i);
    return iq;
}

/**
 * @brief 位置外环（现网：P 分频 + 滞环 + 速度斜坡 PI）。无 #if，行为对齐当前配方展开值。
 */
float outer_ops_pos_tick(motor_context_t *ctx,
                         float theta_fb_rad,
                         float omega_rpm)
{
    uint8_t pos_p_tick = 0u;

    s_pos_p_div++;
    if (s_pos_p_div >= M1_POS_DECIM) {
        s_pos_p_div = 0u;
        pos_p_tick = 1u;
    }
    if (pos_p_tick != 0u) {
        const float err_rad = ctx->theta_ref_rad - theta_fb_rad;
        const uint8_t was_hold = s_pos_err_hold_zone;

        ctx->omega_ref = outer_pos_omega_from_err(err_rad);
        if (s_pos_err_hold_zone != 0u) {
            if (was_hold == 0u) {
                foc_pi_reset(&ctx->pi_speed);
            }
        } else if (was_hold != 0u) {
            foc_pi_bumpless_beta(&ctx->pi_speed, 0.0f,
                                 ctx->omega_ref, omega_rpm,
                                 M1_SPEED_PI_BETA);
        }
    }
    if (s_pos_err_hold_zone != 0u) {
        ctx->iq_ref = 0.0f;
    } else {
        ctx->iq_ref = outer_speed_pi_step(ctx,
                                           outer_omega_ref_ramped(ctx),
                                           omega_rpm,
                                           0u);
    }
    ctx->id_ref = 0.0f;
    return ctx->iq_ref;
}

/**
 * @brief 进入位置环：锁当前角为 θ*。
 */
void outer_ops_pos_enter(motor_context_t *ctx)
{
    motor_outer_arm_position_hold(ctx);
}

/**
 * @brief MIT：现网与位置环同路径（嵌套/并行力矩律未开时）。
 */
float outer_ops_mit_tick(motor_context_t *ctx,
                         float theta_fb_rad,
                         float omega_rpm)
{
    return outer_ops_pos_tick(ctx, theta_fb_rad, omega_rpm);
}

/**
 * @brief 进入 MIT：锁角并清速度积分。
 */
void outer_ops_mit_enter(motor_context_t *ctx)
{
    motor_outer_arm_position_hold(ctx);
    foc_pi_reset(&ctx->pi_speed);
}

/**
 * @brief 按外环模式写 id_ref / iq_ref。DISABLED 不写。
 * @param ctx 控制上下文。不可为 NULL。
 * @note SPEED/TORQUE 若已绑 outer_ops，走表；否则回落本函数 switch。
 */
void motor_outer_loop_tick(motor_context_t *ctx)
{
    float omega_mech_rpm;
    float theta_mech_rad;
    float theta_fb_rad;
    const outer_ops_t *outer;

    if (ctx == NULL) {
        return;
    }

    if (M1_ID_LOCK_CAL_SWEEP) {
        /* Pass0 / VASI：外环不得写 iq_ref/ω_ref（旧版 speed 联调曾导致转子旋转） */
        if (deadband_flow_id_cal_active()) {
            return;
        }
    }

    if (M1_PLL_ENABLE) {
        omega_mech_rpm = motor_current_get_pll_omega_mech_rpm();
    } else {
        omega_mech_rpm = 0.0f;
    }
    theta_mech_rad = motor_current_get_theta_mech_rad();
    theta_fb_rad = motor_current_get_theta_fb_rad();
    outer = motor_outer_active();

    if (M1_EXP_FRAMEWORK_ENABLE) {
        /* Experiment runner: runs every 2 kHz outer tick.
           No-op when nothing is armed. */
        exp_runner_tick(ctx, theta_fb_rad, omega_mech_rpm);
    }

    (void)theta_mech_rad;

    /* 四模式统一走 outer_ops；DISABLED 不写 iq_ref。无宏选路。 */
    if ((ctx->outer_mode != M1_OUTER_DISABLED) &&
        (outer != NULL) && (outer->tick != NULL)) {
        const iref_ops_t *iref = motor_iref_active();

        ctx->iq_ref = outer->tick(ctx, theta_fb_rad, omega_mech_rpm);
        if ((iref != NULL) && (iref->ref != NULL) &&
            ((ctx->outer_mode == M1_OUTER_TORQUE) ||
             (ctx->outer_mode == M1_OUTER_SPEED))) {
            float id_cmd;
            float iq_cmd;

            iref->ref(ctx, ctx->iq_ref, &id_cmd, &iq_cmd);
            ctx->id_ref = id_cmd;
            ctx->iq_ref = iq_cmd;
        } else if ((ctx->outer_mode == M1_OUTER_TORQUE) ||
                   (ctx->outer_mode == M1_OUTER_SPEED)) {
            ctx->id_ref = 0.0f;
        }
        return;
    }

    /* 未绑表时的回落（标定 DISABLED 等）。 */
    if (ctx->outer_mode == M1_OUTER_TORQUE) {
        ctx->iq_ref = outer_clamp_iq_ref(ctx, ctx->iq_cmd);
        ctx->id_ref = 0.0f;
    } else if (ctx->outer_mode == M1_OUTER_SPEED) {
        ctx->iq_ref = outer_ops_speed_tick(ctx, theta_fb_rad, omega_mech_rpm);
        ctx->id_ref = 0.0f;
    } else if (ctx->outer_mode == M1_OUTER_POSITION) {
        ctx->iq_ref = outer_ops_pos_tick(ctx, theta_fb_rad, omega_mech_rpm);
    } else if (ctx->outer_mode == M1_OUTER_MIT) {
        ctx->iq_ref = outer_ops_mit_tick(ctx, theta_fb_rad, omega_mech_rpm);
    }
}

/**
 * @brief 把当前机械角锁成位置指令。
 * @param ctx 控制上下文。不可为 NULL。
 */
void motor_outer_arm_position_hold(motor_context_t *ctx)
{
    if (ctx == NULL) {
        return;
    }

    ctx->theta_ref_rad = motor_current_get_theta_mech_rad();
    ctx->omega_ref = 0.0f;
    /* 现网：滞环 + P 分频常开，进位置/MIT 一律复位。 */
    outer_pos_err_hyst_reset();
    outer_pos_p_decim_arm();
}

/**
 * @brief PLL 就绪后同步斜坡起点和速度 PI。模式已是 SPEED 时 set_mode 会早退，用本函数补。
 * @param ctx 控制上下文。不可为 NULL。
 * @param iq_meas 当前 Iq，单位 A。
 * @param omega_now 当前机械转速，单位 rpm。
 */
void motor_outer_sync_speed_boot(motor_context_t *ctx,
                                  float iq_meas,
                                  float omega_now)
{
    if (ctx == NULL) {
        return;
    }

    if (M1_SPEED_OMEGA_RAMP_ENABLE) {
        s_omega_ramped_rpm = omega_now;
        s_omega_ramp_decel_hold = 0u;
    }
    if (M1_SPEED_IQ_SLEW_ENABLE) {
        s_iq_ref_slew = iq_meas;
    }
    if (ctx->outer_mode == M1_OUTER_SPEED) {
        foc_pi_bumpless_beta(&ctx->pi_speed, iq_meas,
                              ctx->omega_ref, omega_now,
                              M1_SPEED_PI_BETA);
    }
}

/**
 * @brief 只改 ω 斜坡起点，不碰 PI。I-f 交接钉指令速时用。
 * @param omega_rpm 机械转速，单位 rpm。
 */
void motor_outer_set_omega_ramp_rpm(float omega_rpm)
{
    if (M1_SPEED_OMEGA_RAMP_ENABLE) {
        s_omega_ramped_rpm = omega_rpm;
        s_omega_ramp_decel_hold = 0u;
    } else {
        (void)omega_rpm;
    }
}

/**
 * @brief 切换外环模式，并用当前 Iq 与转速做无扰交接。
 * @param ctx 控制上下文。不可为 NULL。
 * @param new_mode 目标模式。
 * @param iq_meas 当前 Iq，单位 A。
 * @param omega_now 当前机械转速，单位 rpm。
 */
void motor_outer_set_mode(motor_context_t *ctx,
                           m1_outer_mode_t new_mode,
                           float iq_meas,
                           float omega_now)
{
    if (ctx == NULL) {
        return;
    }

    if (new_mode == ctx->outer_mode) {
        return;
    }

    switch (new_mode) {
    case M1_OUTER_SPEED:
        foc_pi_bumpless_beta(&ctx->pi_speed, iq_meas,
                              ctx->omega_ref, omega_now,
                              M1_SPEED_PI_BETA);
        s_omega_ramped_rpm = omega_now;
        s_omega_ramp_decel_hold = 0u;
        motor_outer_use(&g_outer_speed);
        if (g_outer_speed.enter != NULL) {
            g_outer_speed.enter(ctx);
        }
        break;

    case M1_OUTER_POSITION:
        foc_pi_bumpless_beta(&ctx->pi_speed, iq_meas, 0.0f, omega_now,
                              M1_SPEED_PI_BETA);
        s_omega_ramped_rpm = omega_now;
        s_omega_ramp_decel_hold = 0u;
        motor_outer_use(&g_outer_pos);
        if (g_outer_pos.enter != NULL) {
            g_outer_pos.enter(ctx);
        }
        break;
    case M1_OUTER_MIT:
        motor_outer_use(&g_outer_mit);
        if (g_outer_mit.enter != NULL) {
            g_outer_mit.enter(ctx);
        }
        break;

    case M1_OUTER_TORQUE:
        foc_pi_reset(&ctx->pi_speed);
        motor_outer_use(&g_outer_torque);
        if (g_outer_torque.enter != NULL) {
            g_outer_torque.enter(ctx);
        }
        break;

    case M1_OUTER_DISABLED:
    default:
        foc_pi_reset(&ctx->pi_speed);
        motor_outer_use(NULL);
        break;
    }

    ctx->outer_mode = new_mode;
    dbg.outer_mode = (uint8_t)new_mode;
}
