/**
 * @file motor_outer_loop.c
 * @date 2026-10-06
 * @brief 外环实现：力矩、速度、位置，以及无扰切换。
 *
 * 本文件写出 iq_ref。电流 PI 和 Park 不在这里。
 * 节拍限制见 motor_outer_loop.h 文件头。
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
#include "speed_ident_flow.h"
#if M1_SPEED_IDENT_ENABLE
#include "speed_ident_module.h"
#endif
#if M1_EXP_FRAMEWORK_ENABLE
#include "experiment/exp_runner.h"
#endif

#if M1_POS_LOOP_ENABLE && M1_POS_ERR_HYST_ENABLE
static void outer_pos_err_hyst_reset(void);
#endif

#if M1_POS_LOOP_ENABLE && (M1_POS_DECIM > 1u)
static void outer_pos_p_decim_arm(void);
#endif

#if M1_SPEED_LOOP_ENABLE

#if M1_SPEED_PROFILE_ENABLE
static uint32_t s_profile_tick;
static uint8_t s_profile_step;
static uint8_t s_profile_repeat_en;
static volatile uint8_t s_profile_ladder_done;

static float motor_speed_profile_rpm_for_step(uint8_t step)
{
    return M1_SPEED_PROFILE_RPM_START +
           (float)step * M1_SPEED_PROFILE_RPM_STEP;
}

static uint8_t motor_speed_profile_step_count(void)
{
    const float steps_f =
        (M1_SPEED_PROFILE_RPM_END - M1_SPEED_PROFILE_RPM_START) /
        M1_SPEED_PROFILE_RPM_STEP + 1.0f;

    if (steps_f < 1.0f) {
        return 1u;
    }
    if (steps_f > 20.0f) {
        return 20u;
    }
    return (uint8_t)(steps_f + 0.5f);
}

/**
 * @brief 重置转速阶梯，从首档写 omega_ref。
 */
void motor_speed_profile_arm(motor_context_t *ctx)
{
    motor_speed_profile_arm_ex(ctx, (M1_SPEED_PROFILE_REPEAT != 0) ? 1u : 0u);
}

void motor_speed_profile_arm_ex(motor_context_t *ctx, uint8_t repeat_en)
{
    if (ctx == NULL) {
        return;
    }

    s_profile_tick = 0u;
    s_profile_step = 0u;
    s_profile_repeat_en = repeat_en;
    s_profile_ladder_done = 0u;
    ctx->omega_ref = motor_speed_profile_rpm_for_step(0u);
    dbg.outer_profile_step = 0u;
}

/**
 * @brief 单次阶梯结束标志。读一次清一次。
 * @return 1=本轮刚结束。
 */
uint8_t motor_speed_profile_consume_ladder_done(void)
{
    if (s_profile_ladder_done == 0u) {
        return 0u;
    }
    s_profile_ladder_done = 0u;
    return 1u;
}

static void motor_speed_profile_tick(motor_context_t *ctx)
{
    const uint32_t hold_ticks =
        (uint32_t)(M1_SPEED_PROFILE_HOLD_S / M1_SPEED_TS_S + 0.5f);
    const uint8_t n_steps = motor_speed_profile_step_count();

    if (hold_ticks == 0u || ctx == NULL) {
        return;
    }

    s_profile_tick++;
    if (s_profile_tick < hold_ticks) {
        return;
    }

    s_profile_tick = 0u;
    if (s_profile_step + 1u < n_steps) {
        s_profile_step++;
    } else if (s_profile_repeat_en != 0u) {
        s_profile_step = 0u;
    } else {
        s_profile_ladder_done = 1u;
        return;
    }

    ctx->omega_ref = motor_speed_profile_rpm_for_step(s_profile_step);
    dbg.outer_profile_step = s_profile_step;
}
#endif /* M1_SPEED_PROFILE_ENABLE */

#if M1_SPEED_REVERSAL_TEST_ENABLE
static uint8_t s_reversal_armed;
static uint32_t s_reversal_tick;
static int8_t s_reversal_sign;

/**
 * @brief 武装正反转速交替试验。
 */
void motor_speed_reversal_arm(motor_context_t *ctx)
{
    if (ctx == NULL) {
        return;
    }

    s_reversal_armed = 1u;
    s_reversal_tick = 0u;
    s_reversal_sign = 1;
    ctx->omega_ref = M1_SPEED_REVERSAL_RPM;
    dbg.outer_profile_step = 0u;
    dbg.open_seq_phase = 201u;
}

static void motor_speed_reversal_tick(motor_context_t *ctx)
{
    const uint32_t hold_ticks =
        (uint32_t)(M1_SPEED_REVERSAL_HOLD_S / M1_SPEED_TS_S + 0.5f);

    if (hold_ticks == 0u || ctx == NULL || s_reversal_armed == 0u) {
        return;
    }

    s_reversal_tick++;
    if (s_reversal_tick < hold_ticks) {
        return;
    }

    s_reversal_tick = 0u;
    if (s_reversal_sign > 0) {
        s_reversal_sign = -1;
        ctx->omega_ref = -M1_SPEED_REVERSAL_RPM;
        dbg.outer_profile_step = 1u;
        dbg.open_seq_phase = 202u;
    } else {
#if M1_SPEED_REVERSAL_REPEAT
        s_reversal_sign = 1;
        ctx->omega_ref = M1_SPEED_REVERSAL_RPM;
        dbg.outer_profile_step = 0u;
        dbg.open_seq_phase = 201u;
#else
        s_reversal_armed = 0u;
        dbg.open_seq_phase = 203u;
#endif
    }
}
#endif /* M1_SPEED_REVERSAL_TEST_ENABLE */

#if M1_POS_STEP_TEST_ENABLE
static uint8_t s_pos_step_armed;
static uint32_t s_pos_step_tick;
static uint8_t s_pos_step_idx;
static float s_pos_theta0;

/** 相对 θ(0) 的绝对目标；多圈段每档 |Δθ|≤2π，便于 1.5~3 s dwell 内跑完 */
static const float s_pos_theta_offset_rad[] = {
    0.0f,           /* hold */
    0.5235988f,     /* +30° */
    0.7853982f,     /* +45° */
    1.0471976f,     /* +60° */
    1.5707963f,     /* +90° */
    2.0943951f,     /* +120° */
    3.1415927f,     /* +180° */
   -0.7853982f,     /* −45° */
   -1.5707963f,     /* −90° */
   -2.0943951f,     /* −120° */
   -3.1415927f,     /* −180° */
    0.0f,           /* 回零（单圈段结束） */
    6.2831853f,     /* +360°  +1 圈 */
   12.5663706f,     /* +720°  +2 圈 */
   18.8495559f,     /* +1080° +3 圈 */
   25.1327412f,     /* +1440° +4 圈 */
   18.8495559f,     /* +1080° −1 圈 */
   12.5663706f,     /* +720°  −1 圈 */
    6.2831853f,     /* +360°  −1 圈 */
    0.0f,           /* 回零（正向多圈结束） */
   -6.2831853f,     /* −360° −1 圈 */
  -12.5663706f,     /* −720° −2 圈 */
  -18.8495559f,     /* −1080°−3 圈 */
  -12.5663706f,     /*  −720° −2 圈 */
   -6.2831853f,     /*  −360° −1 圈 */
    0.0f,           /* 回零（反向多圈结束） */
};

static uint8_t motor_pos_step_count(void)
{
    return (uint8_t)(sizeof(s_pos_theta_offset_rad) / sizeof(s_pos_theta_offset_rad[0]));
}

static float motor_pos_step_fabsf(float x)
{
    return (x < 0.0f) ? -x : x;
}

/** 按本档 |Δθ| 选 dwell：小角 1.5 s，整圈 3 s，≥2 圈 4 s */
static float motor_pos_step_dwell_s(uint8_t step_idx)
{
    float delta_rad;

    if (step_idx == 0u) {
        return M1_POS_STEP_HOLD_S;
    }

    delta_rad = motor_pos_step_fabsf(s_pos_theta_offset_rad[step_idx] -
                                      s_pos_theta_offset_rad[step_idx - 1u]);
    if (delta_rad >= 12.5663706f) {
        return M1_POS_STEP_DWELL_LONG_S;
    }
    if (delta_rad >= 6.2831853f) {
        return M1_POS_STEP_DWELL_MULT_S;
    }
    if (delta_rad >= 3.1415927f) {
        return M1_POS_STEP_DWELL_WIDE_S;
    }
    return M1_POS_STEP_DWELL_S;
}

/**
 * @brief 武装相对当前位置的阶跃试验。
 */
void motor_pos_step_test_arm(motor_context_t *ctx)
{
    if (ctx == NULL) {
        return;
    }

    s_pos_theta0 = motor_current_get_theta_mech_rad();
    s_pos_step_idx = 0u;
    s_pos_step_tick = 0u;
    s_pos_step_armed = 1u;
    ctx->theta_ref_rad = s_pos_theta0 + s_pos_theta_offset_rad[0];
#if M1_POS_ERR_HYST_ENABLE
    outer_pos_err_hyst_reset();
#endif
#if M1_POS_DECIM > 1u
    outer_pos_p_decim_arm();
#endif
    dbg.outer_profile_step = 0u;
    dbg.open_seq_phase = (uint8_t)M1_POS_STEP_SEQ_BASE;
}

static void motor_pos_step_test_tick(motor_context_t *ctx)
{
    const uint8_t n_steps = motor_pos_step_count();
    const float dwell_s = motor_pos_step_dwell_s(s_pos_step_idx);
    const uint32_t hold_ticks =
        (uint32_t)(dwell_s / M1_SPEED_TS_S + 0.5f);

    if (hold_ticks == 0u || ctx == NULL || s_pos_step_armed == 0u) {
        return;
    }

    s_pos_step_tick++;
    if (s_pos_step_tick < hold_ticks) {
        return;
    }

    s_pos_step_tick = 0u;
    if (s_pos_step_idx + 1u < n_steps) {
        s_pos_step_idx++;
        ctx->theta_ref_rad =
            s_pos_theta0 + s_pos_theta_offset_rad[s_pos_step_idx];
#if M1_POS_DECIM > 1u
        outer_pos_p_decim_arm();
#endif
        dbg.outer_profile_step = s_pos_step_idx;
        dbg.open_seq_phase = (uint8_t)(M1_POS_STEP_SEQ_BASE + s_pos_step_idx);
    } else {
            s_pos_step_armed = 0u;
        dbg.open_seq_phase = (uint8_t)(M1_POS_STEP_SEQ_BASE + n_steps);
    }
}
#endif /* M1_POS_STEP_TEST_ENABLE */

#if M1_SPEED_OMEGA_RAMP_ENABLE
static float s_omega_ramped_rpm;
#endif
#if M1_SPEED_IQ_SLEW_ENABLE
static float s_iq_ref_slew;
#endif
#if M1_IF_OBS_EW_CLAMP_ENABLE
static float s_if_obs_ew_guard_s;
#endif
#if M1_IF_OBS_SOFT_BRAKE_ENABLE
static float s_if_obs_soft_brake_s;
static uint8_t s_if_obs_soft_brake_active;
#endif
#if M1_IF_OBS_CRUISE_ENABLE
static uint8_t s_if_cruise_phase;
static float s_if_cruise_phase_s;
static float s_cruise_gate_hold_s;
static float s_cruise_w_prev;
static float s_cruise_domega_abs;
static float s_cruise_domega_win_s;
static uint8_t s_cruise_domega_inited;
static float s_cruise_ew_lpf;
static uint8_t s_cruise_ew_lpf_inited;
static float s_cruise_w_ema;
static float s_cruise_w2_ema;
static uint8_t s_cruise_amp_inited;
static float s_cruise_iq_min_cmd;
static float s_cruise_iq_min_target;
static float s_cruise_iq_max_cmd;
static float s_cruise_iq_max_target;
static float s_cruise_regen_abs_cmd;    /* |regen| 权威，与转向无关 */
static float s_cruise_regen_abs_target;
static float s_cruise_drive_abs_cmd;
#if M1_IF_OBS_CRUISE_STEP_ENABLE
static uint8_t s_cruise_step_phase; /* 0=soak 1..N=表项中 N+1=完 */
static float s_cruise_step_s;
#endif
#if M1_IF_OBS_CRUISE_S3_PROBE_ENABLE
static uint8_t s_s3_probe_phase; /* 0=soak 1=@HI 2=@LO 3=@基 4=完 */
static float s_s3_probe_s;
#endif
#if M1_IF_OBS_DIR_SEQ_ENABLE
static uint8_t s_dir_seq_leg;      /* 0=正转 1=反转 2=完 */
static float s_dir_seq_zero_s;
static float s_dir_seq_coast_s;
static uint8_t s_dir_seq_rearm_req;
static uint8_t s_dir_seq_coast;
static uint8_t s_dir_seq_obs_reset_req; /* 进滑行清一次 SMO，防假速 */
#endif
#endif
#if M1_IF_OBS_DAMP_ENABLE
static float s_damp_omega_lpf;
static uint8_t s_damp_lpf_inited;
#endif

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

#if M1_IF_OBS_SOFT_BRAKE_ENABLE
/** 每拍更新浅刹车窗口；到期恢复 ±PI 全限 */
static void outer_soft_brake_tick(motor_context_t *ctx)
{
    if (ctx == NULL) {
        return;
    }
    if (s_if_obs_soft_brake_s > 0.0f) {
        outer_apply_regen_drive_limits(ctx,
                                         M1_IF_OBS_SOFT_REGEN_IQ_A,
                                         M1_IF_OBS_CRUISE_DRIVE_IQ_A);
        s_if_obs_soft_brake_active = 1u;
        s_if_obs_soft_brake_s -= M1_SPEED_TS_S;
        if (s_if_obs_soft_brake_s < 0.0f) {
            s_if_obs_soft_brake_s = 0.0f;
        }
    } else if (s_if_obs_soft_brake_active != 0u) {
        ctx->pi_speed.out_min = M1_SPEED_PI_OUT_MIN;
        ctx->pi_speed.out_max = M1_SPEED_PI_OUT_MAX;
        ctx->pi_speed.int_min = M1_SPEED_PI_INT_MIN;
        ctx->pi_speed.int_max = M1_SPEED_PI_INT_MAX;
        if (ctx->pi_speed.integrator > ctx->pi_speed.int_max) {
            ctx->pi_speed.integrator = ctx->pi_speed.int_max;
        } else if (ctx->pi_speed.integrator < ctx->pi_speed.int_min) {
            ctx->pi_speed.integrator = ctx->pi_speed.int_min;
        }
        s_if_obs_soft_brake_active = 0u;
    }
}
#endif

#if M1_SPEED_OMEGA_RAMP_ENABLE
static float outer_omega_ref_ramped(motor_context_t *ctx)
{
    const float target = ctx->omega_ref;
    const float step_up = M1_SPEED_OMEGA_RAMP_RPM_S * M1_SPEED_TS_S;
#if defined(M1_SPEED_OMEGA_RAMP_DECEL_RPM_S)
    const float step_dn = M1_SPEED_OMEGA_RAMP_DECEL_RPM_S * M1_SPEED_TS_S;
#else
    const float step_dn = step_up;
#endif
    float delta = target - s_omega_ramped_rpm;

    if (delta > step_up) {
        s_omega_ramped_rpm += step_up;
    } else if (delta < -step_dn) {
        s_omega_ramped_rpm -= step_dn;
    } else {
        s_omega_ramped_rpm = target;
    }
    return s_omega_ramped_rpm;
}
#endif

static float outer_speed_pi_step(motor_context_t *ctx,
                                  float omega_ref,
                                  float omega_fb,
                                  uint8_t hold_i)
{
    float fb = omega_fb;
    float ref_pi = omega_ref;
    float fb_pi;
#if M1_IF_OBS_SOFT_BRAKE_ENABLE
    outer_soft_brake_tick(ctx);
#endif
#if M1_IF_ENABLE && M1_IF_TO_OBS_ENABLE
    /*
     * I/F 正/反转拖动均为 +Iq（2121/2127）。
     * 速度环用 |ω| 误差：还不够快 → +Iq；勿用有符号 e=ref−fb
     * （反转时 e<0 会把 Iq 拧向 −regen，等效把扭矩拧反）。
     */
    if (ref_pi < 0.0f) {
        ref_pi = -ref_pi;
    }
    if (fb < 0.0f) {
        fb = -fb;
    }
#endif
    fb_pi = fb;
#if M1_IF_OBS_EW_CLAMP_ENABLE
    /* 切入瞬态：限 |e_ω|（已在 |ω| 域） */
    if (s_if_obs_ew_guard_s > 0.0f) {
        float ew = ref_pi - fb_pi;
        const float lim = M1_IF_OBS_EW_CLAMP_RPM;

        if (ew > lim) {
            ew = lim;
        } else if (ew < -lim) {
            ew = -lim;
        }
        fb_pi = ref_pi - ew;
        s_if_obs_ew_guard_s -= M1_SPEED_TS_S;
        if (s_if_obs_ew_guard_s < 0.0f) {
            s_if_obs_ew_guard_s = 0.0f;
        }
    }
#endif
    float iq = foc_pi_step_beta_hold_i(&ctx->pi_speed,
                                        ref_pi,
                                        fb_pi,
                                        M1_SPEED_PI_BETA,
                                        hold_i);
#if M1_IF_OBS_DAMP_ENABLE
    /* 阻尼：HP=交流粘滞；绝对模式仅对照用（浅刹下已证实有害） */
    {
        if (s_damp_lpf_inited == 0u) {
            s_damp_omega_lpf = omega_fb;
            s_damp_lpf_inited = 1u;
        } else {
            const float wc = 6.28318530718f * M1_IF_OBS_DAMP_LPF_HZ;
            const float a = (wc * M1_SPEED_TS_S) / (1.0f + wc * M1_SPEED_TS_S);
            s_damp_omega_lpf += a * (omega_fb - s_damp_omega_lpf);
        }
#if M1_IF_OBS_DAMP_HP_ENABLE
        iq -= M1_IF_OBS_DAMP_BD * (omega_fb - s_damp_omega_lpf);
#else
        iq -= M1_IF_OBS_DAMP_BD * s_damp_omega_lpf;
#endif
    }
#endif
#if M1_SPEED_IQ_SLEW_ENABLE
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
#else
    return outer_clamp_iq_ref(ctx, iq);
#endif
}

#if M1_POS_LOOP_ENABLE
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

#if M1_POS_ERR_HYST_ENABLE
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
#endif /* M1_POS_ERR_HYST_ENABLE */

#if M1_POS_DECIM > 1u
static uint8_t s_pos_p_div;

static void outer_pos_p_decim_arm(void)
{
    s_pos_p_div = M1_POS_DECIM;
}
#endif /* M1_POS_DECIM > 1u */
#endif /* M1_POS_LOOP_ENABLE */

/**
 * @brief 装速度 PI，默认 DISABLED。
 * @param ctx 控制上下文。不可为 NULL。
 */
void motor_outer_loop_init(motor_context_t *ctx)
{
    if (ctx == NULL) {
        return;
    }

    foc_pi_init(&ctx->pi_speed,
                M1_SPEED_PI_KP,
                M1_SPEED_PI_KI,
                M1_SPEED_PI_OUT_MIN,
                M1_SPEED_PI_OUT_MAX,
                M1_SPEED_PI_INT_MIN,
                M1_SPEED_PI_INT_MAX);

    ctx->outer_mode = M1_OUTER_DISABLED;
    ctx->omega_ref  = 0.0f;
    ctx->theta_ref_rad = 0.0f;
    ctx->iq_cmd     = 0.0f;
    ctx->iq_ff      = 0.0f;
    ctx->iq_fric_ff = 0.0f;
    ctx->traj_vel_rpm = 0.0f;
#if M1_POS_LOOP_ENABLE && M1_POS_ERR_HYST_ENABLE
    outer_pos_err_hyst_reset();
#endif
#if M1_POS_LOOP_ENABLE && (M1_POS_DECIM > 1u)
    outer_pos_p_decim_arm();
#endif
#if M1_SPEED_OMEGA_RAMP_ENABLE
    s_omega_ramped_rpm = 0.0f;
#endif
#if M1_SPEED_IQ_SLEW_ENABLE
    s_iq_ref_slew = 0.0f;
#endif
#if M1_IF_OBS_EW_CLAMP_ENABLE
    s_if_obs_ew_guard_s = 0.0f;
#endif
#if M1_IF_OBS_SOFT_BRAKE_ENABLE
    s_if_obs_soft_brake_s = 0.0f;
    s_if_obs_soft_brake_active = 0u;
#endif
#if M1_IF_OBS_CRUISE_ENABLE
    s_if_cruise_phase = 0u;
    s_if_cruise_phase_s = 0.0f;
    s_cruise_gate_hold_s = 0.0f;
    s_cruise_w_prev = 0.0f;
    s_cruise_domega_abs = 0.0f;
    s_cruise_domega_win_s = 0.0f;
    s_cruise_domega_inited = 0u;
    s_cruise_ew_lpf = 0.0f;
    s_cruise_ew_lpf_inited = 0u;
    s_cruise_w_ema = 0.0f;
    s_cruise_w2_ema = 0.0f;
    s_cruise_amp_inited = 0u;
    s_cruise_iq_min_cmd = M1_SPEED_PI_OUT_MIN;
    s_cruise_iq_min_target = M1_SPEED_PI_OUT_MIN;
    s_cruise_iq_max_cmd = M1_SPEED_PI_OUT_MAX;
    s_cruise_iq_max_target = M1_SPEED_PI_OUT_MAX;
    s_cruise_regen_abs_cmd = M1_IF_OBS_SOFT_REGEN_IQ_A;
    s_cruise_regen_abs_target = M1_IF_OBS_SOFT_REGEN_IQ_A;
    s_cruise_drive_abs_cmd = M1_IF_OBS_CRUISE_DRIVE_IQ_A;
#if M1_IF_OBS_CRUISE_STEP_ENABLE
    s_cruise_step_phase = 0u;
    s_cruise_step_s = 0.0f;
#endif
#if M1_IF_OBS_CRUISE_S3_PROBE_ENABLE
    s_s3_probe_phase = 0u;
    s_s3_probe_s = 0.0f;
#endif
#if M1_IF_OBS_DIR_SEQ_ENABLE
    s_dir_seq_leg = 0u;
    s_dir_seq_zero_s = 0.0f;
    s_dir_seq_coast_s = 0.0f;
    s_dir_seq_rearm_req = 0u;
    s_dir_seq_coast = 0u;
    s_dir_seq_obs_reset_req = 0u;
#endif
#endif
#if M1_IF_OBS_DAMP_ENABLE
    s_damp_omega_lpf = 0.0f;
    s_damp_lpf_inited = 0u;
#endif
}

#if M1_IF_OBS_EW_CLAMP_ENABLE
/**
 * @brief I-f 交观测后武装转速误差限幅计时。
 */
void motor_outer_if_obs_ew_guard_arm(void)
{
    s_if_obs_ew_guard_s = M1_IF_OBS_EW_CLAMP_S;
}
#endif

#if M1_IF_OBS_SOFT_BRAKE_ENABLE
void motor_outer_if_obs_soft_brake_arm(motor_context_t *ctx)
{
    s_if_obs_soft_brake_s = M1_IF_OBS_SOFT_BRAKE_S;
    s_if_obs_soft_brake_active = 1u;
    if (ctx != NULL) {
        outer_apply_regen_drive_limits(ctx,
                                         M1_IF_OBS_SOFT_REGEN_IQ_A,
                                         M1_IF_OBS_CRUISE_DRIVE_IQ_A);
        outer_soft_brake_snap_iq(ctx);
    }
}
#endif

#if M1_IF_OBS_CRUISE_ENABLE
/* 0=未武装 1=浅刹巡航 2=第一档 iq_min 3=正常 PI */
static void outer_cruise_bumpless(motor_context_t *ctx, float omega_fb)
{
    foc_pi_bumpless_beta(&ctx->pi_speed,
                         ctx->iq_ref,
                         ctx->omega_ref,
                         omega_fb,
                         M1_SPEED_PI_BETA);
}

static void outer_cruise_apply_iq_minmax(motor_context_t *ctx,
                                          float iq_min,
                                          float iq_max)
{
    ctx->pi_speed.out_max = iq_max;
    ctx->pi_speed.out_min = iq_min;
    ctx->pi_speed.int_max = iq_max;
    ctx->pi_speed.int_min = iq_min;
    if (ctx->pi_speed.integrator > ctx->pi_speed.int_max) {
        ctx->pi_speed.integrator = ctx->pi_speed.int_max;
    } else if (ctx->pi_speed.integrator < ctx->pi_speed.int_min) {
        ctx->pi_speed.integrator = ctx->pi_speed.int_min;
    }
}

#if M1_IF_OBS_SOFT_BRAKE_ENABLE
/** @brief ①：保持浅刹车不因定时到期跳回大权限 */
static void outer_cruise_hold_soft_brake(motor_context_t *ctx)
{
    s_if_obs_soft_brake_s = 1.0f;
    s_if_obs_soft_brake_active = 1u;
    if (ctx != NULL) {
        outer_apply_regen_drive_limits(ctx,
                                         M1_IF_OBS_SOFT_REGEN_IQ_A,
                                         s_cruise_drive_abs_cmd);
    }
}
#endif

#if M1_IF_OBS_CRUISE_GATE_ENABLE
/**
 * @brief 抗抖+晃幅门控：|eω| LPF、长窗 |dω|、EMA std/峰峰代理。
 * @return 1=hold 达标，可升权威
 */
static uint8_t outer_cruise_gate_step(motor_context_t *ctx,
                                       float omega_fb,
                                       float dt)
{
    float ew;
    float aew;
    uint8_t ok;
    float std_w = 1.0e6f;
    float ptp_proxy = 1.0e6f;

    if ((ctx == NULL) || (dt <= 0.0f)) {
        return 0u;
    }

    if (s_cruise_domega_inited == 0u) {
        s_cruise_w_prev = omega_fb;
        s_cruise_domega_win_s = 0.0f;
        s_cruise_domega_abs = 1.0e6f;
        s_cruise_domega_inited = 1u;
    } else {
        s_cruise_domega_win_s += dt;
        if (s_cruise_domega_win_s >= M1_IF_OBS_CRUISE_GATE_DOMEGA_WIN_S) {
            s_cruise_domega_abs =
                (omega_fb - s_cruise_w_prev) / s_cruise_domega_win_s;
            if (s_cruise_domega_abs < 0.0f) {
                s_cruise_domega_abs = -s_cruise_domega_abs;
            }
            s_cruise_w_prev = omega_fb;
            s_cruise_domega_win_s = 0.0f;
        }
    }

    ew = ctx->omega_ref - omega_fb;
    aew = (ew < 0.0f) ? -ew : ew;
    if (s_cruise_ew_lpf_inited == 0u) {
        s_cruise_ew_lpf = aew;
        s_cruise_ew_lpf_inited = 1u;
    } else {
        const float wc = 6.28318530718f * M1_IF_OBS_CRUISE_GATE_EW_LPF_HZ;
        const float a = (wc * dt) / (1.0f + wc * dt);

        s_cruise_ew_lpf += a * (aew - s_cruise_ew_lpf);
    }

#if M1_IF_OBS_CRUISE_AMP_GATE_ENABLE
    {
        const float wc = 6.28318530718f * M1_IF_OBS_CRUISE_AMP_EMA_HZ;
        const float a = (wc * dt) / (1.0f + wc * dt);
        float var;

        if (s_cruise_amp_inited == 0u) {
            s_cruise_w_ema = omega_fb;
            s_cruise_w2_ema = omega_fb * omega_fb;
            s_cruise_amp_inited = 1u;
        } else {
            s_cruise_w_ema += a * (omega_fb - s_cruise_w_ema);
            s_cruise_w2_ema += a * ((omega_fb * omega_fb) - s_cruise_w2_ema);
        }
        var = s_cruise_w2_ema - (s_cruise_w_ema * s_cruise_w_ema);
        if (var < 0.0f) {
            var = 0.0f;
        }
        std_w = sqrtf(var);
        ptp_proxy = 2.5f * std_w;
    }
#endif

    ok = (s_cruise_ew_lpf <= M1_IF_OBS_CRUISE_GATE_ERR_RPM) &&
         (s_cruise_domega_abs <= M1_IF_OBS_CRUISE_GATE_DOMEGA_MAX);
#if M1_IF_OBS_CRUISE_AMP_GATE_ENABLE
    ok = (ok != 0u) &&
         (std_w <= M1_IF_OBS_CRUISE_AMP_STD_MAX) &&
         (ptp_proxy <= M1_IF_OBS_CRUISE_AMP_PTP_MAX) &&
         (s_cruise_ew_lpf <= M1_IF_OBS_CRUISE_AMP_EW_MAX);
#endif

    if (ok != 0u) {
        s_cruise_gate_hold_s += dt;
    } else {
        s_cruise_gate_hold_s -= M1_IF_OBS_CRUISE_GATE_FAIL_LEAK * dt;
        if (s_cruise_gate_hold_s < 0.0f) {
            s_cruise_gate_hold_s = 0.0f;
        }
    }

    return (s_cruise_gate_hold_s >= M1_IF_OBS_CRUISE_GATE_HOLD_S) ? 1u : 0u;
}
#endif

/** @brief 斜坡放开 |regen|，再按 DIR 写回 PI 限幅 */
static void outer_cruise_iq_min_slew_apply(motor_context_t *ctx, float dt)
{
    float step;
    float delta;

    if ((ctx == NULL) || (dt <= 0.0f)) {
        return;
    }

    step = M1_IF_OBS_CRUISE_IQ_MIN_SLEW_A_S * dt;
    delta = s_cruise_regen_abs_target - s_cruise_regen_abs_cmd;
    if (delta > step) {
        s_cruise_regen_abs_cmd += step;
    } else if (delta < -step) {
        s_cruise_regen_abs_cmd -= step;
    } else {
        s_cruise_regen_abs_cmd = s_cruise_regen_abs_target;
    }
    outer_apply_regen_drive_limits(ctx,
                                     s_cruise_regen_abs_cmd,
                                     s_cruise_drive_abs_cmd);
    outer_iq_lims_from_regen_drive(s_cruise_regen_abs_cmd,
                                     s_cruise_drive_abs_cmd,
                                     &s_cruise_iq_min_cmd,
                                     &s_cruise_iq_max_cmd);
    s_cruise_iq_min_target = s_cruise_iq_min_cmd;
    s_cruise_iq_max_target = s_cruise_iq_max_cmd;
}

/** @brief 1=|regen| 斜坡已到位 */
static uint8_t outer_cruise_iq_auth_ready(void)
{
    return (s_cruise_regen_abs_cmd >=
            (s_cruise_regen_abs_target - 0.02f))
               ? 1u
               : 0u;
}

#if M1_IF_OBS_CRUISE_STEP_ENABLE
/* soak 后：HI→基→LO→HI→LO→基（无弱磁带内大阶跃 + 减速验 regen） */
enum {
    OUTER_CRUISE_STEP_N = 6u,
    OUTER_CRUISE_STEP_DONE = 7u /* phase：0=soak, 1..6=表项, 7=完 */
};

static const float s_cruise_step_rpm[OUTER_CRUISE_STEP_N] = {
    M1_IF_OBS_CRUISE_STEP_HI_RPM, /* 1300 */
    M1_IF_OBS_CRUISE_RPM,         /* 1000 */
    M1_IF_OBS_CRUISE_STEP_LO_RPM, /* 900 */
    M1_IF_OBS_CRUISE_STEP_HI_RPM, /* 1300 */
    M1_IF_OBS_CRUISE_STEP_LO_RPM, /* 900 */
    M1_IF_OBS_CRUISE_RPM,         /* 1000 */
};

static void outer_cruise_step_apply(motor_context_t *ctx, float rpm, uint8_t idx)
{
    ctx->omega_ref = rpm;
#if M1_SPEED_OMEGA_RAMP_ENABLE
    /* 硬阶跃：把斜坡状态钉到目标，否则 200rpm/s 会抹平响应 */
    motor_outer_set_omega_ramp_rpm(rpm);
#endif
    dbg.open_seq_phase = (uint8_t)(EXP_MARK_CRUISE_STEP_BASE + idx); /* 233..238 */
}

/** @brief 巡航后 ω_ref 硬阶跃表（900..1300） */
static void outer_cruise_step_tick(motor_context_t *ctx, float dt)
{
    uint8_t idx;

    if ((ctx == NULL) || (dt <= 0.0f) ||
        (s_cruise_step_phase >= OUTER_CRUISE_STEP_DONE)) {
        return;
    }

    /* 须已到②且限权斜坡到位 */
    if (s_if_cruise_phase < 2u) {
        return;
    }
    if (outer_cruise_iq_auth_ready() == 0u) {
        return;
    }

    s_cruise_step_s += dt;

    if (s_cruise_step_phase == 0u) {
        if (s_cruise_step_s < M1_IF_OBS_CRUISE_STEP_SOAK_S) {
            return;
        }
        outer_cruise_step_apply(ctx, s_cruise_step_rpm[0], 0u);
        s_cruise_step_phase = 1u;
        s_cruise_step_s = 0.0f;
        return;
    }

    if (s_cruise_step_s < M1_IF_OBS_CRUISE_STEP_HOLD_S) {
        return;
    }

    /* phase 1..N 对应已施加表项 [phase-1]；满 hold 后进下一项或结束 */
    if (s_cruise_step_phase >= OUTER_CRUISE_STEP_N) {
        s_cruise_step_phase = OUTER_CRUISE_STEP_DONE;
        dbg.open_seq_phase = EXP_MARK_SPEED_STEP_TABLE_END; /* 阶跃表结束（249 留给③） */
        return;
    }

    idx = s_cruise_step_phase; /* 下一表项下标 */
    outer_cruise_step_apply(ctx, s_cruise_step_rpm[idx], idx);
    s_cruise_step_phase = (uint8_t)(idx + 1u);
    s_cruise_step_s = 0.0f;
}
#endif

#if M1_IF_OBS_CRUISE_S3_PROBE_ENABLE
/** @brief 硬阶跃钉 ω_ref（与巡航表同口径） */
static void outer_cruise_s3_probe_apply(motor_context_t *ctx, float rpm, uint8_t seq)
{
    ctx->omega_ref = rpm;
#if M1_SPEED_OMEGA_RAMP_ENABLE
    motor_outer_set_omega_ramp_rpm(rpm);
#endif
    dbg.open_seq_phase = seq;
}

/**
 * @brief ③ 到位后硬减速探针：基→HI→LO→基，逼 Iq 贴 STAGE3（1714：仅 1000→900 不够）
 */
static void outer_cruise_s3_probe_tick(motor_context_t *ctx, float dt)
{
    if ((ctx == NULL) || (dt <= 0.0f) || (s_s3_probe_phase >= 4u)) {
        return;
    }
    if (s_if_cruise_phase < 3u) {
        return;
    }
    /* 限权须已斜到③目标 */
    if (outer_cruise_iq_auth_ready() == 0u) {
        return;
    }

    s_s3_probe_s += dt;

    if (s_s3_probe_phase == 0u) {
        if (s_s3_probe_s < M1_IF_OBS_CRUISE_S3_PROBE_SOAK_S) {
            dbg.open_seq_phase = EXP_MARK_CRUISE_STAGE3_WAIT; /* ③ 到位，等探针 */
            return;
        }
        /* 先拉到 HI，再砸向 LO，形成大减速 */
        outer_cruise_s3_probe_apply(ctx, M1_IF_OBS_CRUISE_STEP_HI_RPM,
                                    EXP_MARK_S3_PROBE_HI);
        s_s3_probe_phase = 1u;
        s_s3_probe_s = 0.0f;
        return;
    }

    if (s_s3_probe_s < M1_IF_OBS_CRUISE_S3_PROBE_HOLD_S) {
        return;
    }

    if (s_s3_probe_phase == 1u) {
        outer_cruise_s3_probe_apply(ctx, M1_IF_OBS_CRUISE_STEP_LO_RPM,
                                    EXP_MARK_S3_PROBE_LO); /* HI→LO */
        s_s3_probe_phase = 2u;
        s_s3_probe_s = 0.0f;
    } else if (s_s3_probe_phase == 2u) {
        outer_cruise_s3_probe_apply(ctx, M1_IF_OBS_CRUISE_RPM,
                                    EXP_MARK_S3_PROBE_BASE); /* →基 */
        s_s3_probe_phase = 3u;
        s_s3_probe_s = 0.0f;
    } else if (s_s3_probe_phase == 3u) {
        s_s3_probe_phase = 4u;
        dbg.open_seq_phase = EXP_MARK_OUTER_S3_PROBE_DONE; /* s3 probe done */
    }
}
#endif

/**
 * @brief 武装巡航：浅刹后再逐步放开负流。
 * @param omega_fb 当前机械转速，单位 rpm。
 */
void motor_outer_if_obs_cruise_arm(motor_context_t *ctx, float omega_fb)
{
    float cruise_rpm;

    if (ctx == NULL) {
        return;
    }

    s_if_cruise_phase = 1u;
    s_if_cruise_phase_s = 0.0f;
    s_cruise_gate_hold_s = 0.0f;
    s_cruise_domega_inited = 0u;
    s_cruise_domega_abs = 1.0e6f;
    s_cruise_domega_win_s = 0.0f;
    s_cruise_w_prev = omega_fb;
    s_cruise_ew_lpf = 0.0f;
    s_cruise_ew_lpf_inited = 0u;
    s_cruise_amp_inited = 0u;
    s_cruise_w_ema = omega_fb;
    s_cruise_w2_ema = omega_fb * omega_fb;
    s_cruise_drive_abs_cmd = M1_IF_OBS_CRUISE_DRIVE_IQ_A;
    s_cruise_regen_abs_cmd = M1_IF_OBS_SOFT_REGEN_IQ_A;
    s_cruise_regen_abs_target = M1_IF_OBS_SOFT_REGEN_IQ_A;
    outer_iq_lims_from_regen_drive(s_cruise_regen_abs_cmd,
                                     s_cruise_drive_abs_cmd,
                                     &s_cruise_iq_min_cmd,
                                     &s_cruise_iq_max_cmd);
    s_cruise_iq_min_target = s_cruise_iq_min_cmd;
    s_cruise_iq_max_target = s_cruise_iq_max_cmd;
#if M1_IF_OBS_CRUISE_STEP_ENABLE
    s_cruise_step_phase = 0u;
    s_cruise_step_s = 0.0f;
#endif
#if M1_IF_OBS_CRUISE_S3_PROBE_ENABLE
    s_s3_probe_phase = 0u;
    s_s3_probe_s = 0.0f;
#endif
#if M1_IF_OBS_DIR_SEQ_ENABLE
    s_dir_seq_coast = 0u;
    s_dir_seq_zero_s = 0.0f;
    s_dir_seq_coast_s = 0.0f;
    s_dir_seq_rearm_req = 0u;
#endif

#if M1_IF_OBS_SOFT_BRAKE_ENABLE
    outer_cruise_hold_soft_brake(ctx);
#endif

    outer_cruise_bumpless(ctx, omega_fb);
#if M1_SPEED_OMEGA_RAMP_ENABLE
    motor_outer_set_omega_ramp_rpm(ctx->omega_ref);
#endif
#if M1_IF_OBS_DIR_SEQ_ENABLE
    /* 巡航目标跟当前 I/F 运行时目标（反转腿为 −1000） */
    cruise_rpm = motor_if_get_target_rpm();
#else
    cruise_rpm = M1_IF_OBS_CRUISE_RPM;
#endif
    ctx->omega_ref = cruise_rpm;
    dbg.open_seq_phase = EXP_MARK_CRUISE_SOFT_BRAKE; /* cruise① soft-brake */
}

/**
 * @brief 推进巡航门控。
 * @param omega_fb 当前机械转速，单位 rpm。
 * @param dt 外环节拍，单位 s。
 */
void motor_outer_if_obs_cruise_tick(motor_context_t *ctx, float omega_fb, float dt)
{
    if ((ctx == NULL) || (s_if_cruise_phase == 0u)) {
        return;
    }
    if (dt <= 0.0f) {
        return;
    }

#if M1_IF_OBS_DIR_SEQ_ENABLE
    /* 5=滑行不管  6=近零待反起  8=反转腿结束钉住 */
    if (s_if_cruise_phase == 5u) {
        float w_enc = motor_current_get_pll_omega_mech_rpm();
#if M1_EMF_SMO_ENABLE
        float emag = dbg.obs_smo_emag;
#else
        float emag = dbg.obs_emag;
#endif
        uint8_t stopped = 0u;

        ctx->omega_ref = 0.0f;
        dbg.open_seq_phase = EXP_MARK_SPEED_IDENT_RUN;
        s_dir_seq_coast_s += dt;

        if (w_enc < 0.0f) {
            w_enc = -w_enc;
        }
        if (emag < 0.0f) {
            emag = -emag;
        }

        /*
         * 2144：机械已停，但 Iq=0 后 ω̂ 假挂 ~400，不能当近零。
         * enc 可信则信 enc；否则看 emag 噪声地板；再不行超时强制反起。
         */
        if (w_enc > 30.0f) {
            stopped = (w_enc <= M1_IF_OBS_DIR_SEQ_ZERO_RPM) ? 1u : 0u;
        } else {
            stopped = (emag <= M1_IF_OBS_DIR_SEQ_EMAG_STOP) ? 1u : 0u;
        }
        if (s_dir_seq_coast_s >= M1_IF_OBS_DIR_SEQ_COAST_MAX_S) {
            stopped = 1u;
        }

        if (s_dir_seq_coast_s < M1_IF_OBS_DIR_SEQ_COAST_MIN_S) {
            s_dir_seq_zero_s = 0.0f;
            (void)omega_fb;
            return;
        }
        if (stopped != 0u) {
            s_dir_seq_zero_s += dt;
            if (s_dir_seq_zero_s >= M1_IF_OBS_DIR_SEQ_ZERO_HOLD_S) {
                s_dir_seq_rearm_req = 1u;
                s_if_cruise_phase = 6u;
                dbg.open_seq_phase = EXP_MARK_SPEED_IDENT_BLEND;
            }
        } else {
            s_dir_seq_zero_s = 0.0f;
        }
        (void)omega_fb;
        return;
    }
    if (s_if_cruise_phase == 6u) {
        ctx->omega_ref = 0.0f;
        dbg.open_seq_phase = EXP_MARK_SPEED_IDENT_BLEND;
        return;
    }
    if (s_if_cruise_phase == 8u) {
        dbg.open_seq_phase = EXP_MARK_SPEED_IDENT_DONE;
        return;
    }
#endif

    /* 权威 FSM 完成后仍可跑转速阶跃 */
    if (s_if_cruise_phase >= 4u) {
#if M1_IF_OBS_CRUISE_STEP_ENABLE
        outer_cruise_step_tick(ctx, dt);
#else
        (void)omega_fb;
#endif
        return;
    }

    s_if_cruise_phase_s += dt;

    if (s_if_cruise_phase == 1u) {
#if M1_IF_OBS_SOFT_BRAKE_ENABLE
        outer_cruise_hold_soft_brake(ctx);
#endif
        dbg.open_seq_phase = EXP_MARK_CRUISE_SOFT_BRAKE;

#if M1_IF_OBS_DIR_SEQ_ENABLE
        {
            const float hold_s = (s_dir_seq_leg == 0u) ?
                M1_IF_OBS_DIR_SEQ_FWD_HOLD_S : M1_IF_OBS_DIR_SEQ_REV_HOLD_S;

            if (s_if_cruise_phase_s >= hold_s) {
                if (s_dir_seq_leg == 0u) {
                    /* 正转站稳 → Iq=0 滑行，等近零 */
                    s_if_cruise_phase = 5u;
                    s_dir_seq_coast = 1u;
                    s_dir_seq_zero_s = 0.0f;
                    s_dir_seq_coast_s = 0.0f;
                    s_dir_seq_rearm_req = 0u;
                    s_dir_seq_obs_reset_req = 1u; /* 清 SMO 假速 */
#if M1_IF_OBS_SOFT_BRAKE_ENABLE
                    s_if_obs_soft_brake_s = 0.0f;
                    s_if_obs_soft_brake_active = 0u;
#endif
                    ctx->omega_ref = 0.0f;
                    dbg.open_seq_phase = EXP_MARK_SPEED_IDENT_RUN;
                    return;
                }
                /* 反转站稳 → 结束 */
                s_if_cruise_phase = 8u;
                s_dir_seq_leg = 2u;
                dbg.open_seq_phase = EXP_MARK_SPEED_IDENT_DONE;
                return;
            }
        }
#endif

#if M1_IF_OBS_CRUISE_LOCK_STAGE1_ENABLE
#if M1_IF_OBS_CRUISE_STEP_ENABLE
        outer_cruise_step_tick(ctx, dt);
#else
        (void)omega_fb;
#endif
        return;
#else
#if M1_IF_OBS_CRUISE_GATE_ENABLE
        (void)outer_cruise_gate_step(ctx, omega_fb, dt);
        if (s_if_cruise_phase_s < M1_IF_OBS_CRUISE_GATE_T_MIN_S) {
            s_cruise_gate_hold_s = 0.0f;
            return;
        }
        if (s_cruise_gate_hold_s < M1_IF_OBS_CRUISE_GATE_HOLD_S) {
            return;
        }
#else
        if (s_if_cruise_phase_s < M1_IF_OBS_CRUISE_STAGE1_S) {
            return;
        }
#endif
#if M1_IF_OBS_SOFT_BRAKE_ENABLE
        s_if_obs_soft_brake_s = 0.0f;
        s_if_obs_soft_brake_active = 0u;
#endif
        /* ②：|regen| 从浅刹斜到 STAGE2 */
        s_cruise_regen_abs_cmd = M1_IF_OBS_SOFT_REGEN_IQ_A;
        s_cruise_regen_abs_target = M1_IF_OBS_CRUISE_STAGE2_REGEN_IQ_A;
        s_cruise_drive_abs_cmd = M1_IF_OBS_CRUISE_DRIVE_IQ_A;
        outer_apply_regen_drive_limits(ctx,
                                         s_cruise_regen_abs_cmd,
                                         s_cruise_drive_abs_cmd);
        outer_iq_lims_from_regen_drive(s_cruise_regen_abs_cmd,
                                         s_cruise_drive_abs_cmd,
                                         &s_cruise_iq_min_cmd,
                                         &s_cruise_iq_max_cmd);
        s_cruise_iq_min_target = s_cruise_iq_min_cmd;
        s_cruise_iq_max_target = s_cruise_iq_max_cmd;
        outer_cruise_bumpless(ctx, omega_fb);
        s_if_cruise_phase = 2u;
        s_if_cruise_phase_s = 0.0f;
        s_cruise_gate_hold_s = 0.0f;
#if M1_IF_OBS_CRUISE_STEP_ENABLE
        s_cruise_step_s = 0.0f;
#endif
        dbg.open_seq_phase = EXP_MARK_CRUISE_AUTH_RAMP; /* ② 限权斜坡 */
#endif /* !LOCK_STAGE1 */
    } else if (s_if_cruise_phase == 2u) {
        outer_cruise_iq_min_slew_apply(ctx, dt);
#if M1_IF_OBS_CRUISE_STEP_ENABLE
        /* step 未开时保持 247；阶跃中用 250–254 */
        if (s_cruise_step_phase == 0u) {
            dbg.open_seq_phase = EXP_MARK_CRUISE_AUTH_RAMP;
        }
        outer_cruise_step_tick(ctx, dt);
#else
        dbg.open_seq_phase = EXP_MARK_CRUISE_AUTH_RAMP;
#endif
#if M1_IF_OBS_CRUISE_LOCK_STAGE2_ENABLE
        (void)omega_fb;
        return;
#else
#if M1_IF_OBS_CRUISE_STEP_ENABLE
        /* 1652：整表阶跃在②跑完再进③，避免 1300 hold 中途放宽 regen */
        if (s_cruise_step_phase < OUTER_CRUISE_STEP_DONE) {
            s_cruise_gate_hold_s = 0.0f;
            (void)omega_fb;
            return;
        }
#endif
#if M1_IF_OBS_CRUISE_GATE_ENABLE
        /* 斜坡未到目标前不升③ */
        if (outer_cruise_iq_auth_ready() == 0u) {
            s_cruise_gate_hold_s = 0.0f;
            (void)outer_cruise_gate_step(ctx, omega_fb, dt);
            return;
        }
        (void)outer_cruise_gate_step(ctx, omega_fb, dt);
        if (s_if_cruise_phase_s < M1_IF_OBS_CRUISE_GATE_T_MIN2_S) {
            s_cruise_gate_hold_s = 0.0f;
            return;
        }
        if (s_cruise_gate_hold_s < M1_IF_OBS_CRUISE_GATE_HOLD_S) {
            return;
        }
#else
        if (s_if_cruise_phase_s < M1_IF_OBS_CRUISE_STAGE2_S) {
            return;
        }
#endif
        s_cruise_regen_abs_target = M1_IF_OBS_CRUISE_STAGE3_REGEN_IQ_A;
        outer_cruise_bumpless(ctx, omega_fb);
        s_if_cruise_phase = 3u;
        s_if_cruise_phase_s = 0.0f;
        s_cruise_gate_hold_s = 0.0f;
#if M1_IF_OBS_CRUISE_S3_PROBE_ENABLE
        s_s3_probe_phase = 0u;
        s_s3_probe_s = 0.0f;
#endif
        dbg.open_seq_phase = EXP_MARK_CRUISE_STAGE3_WAIT; /* ③ |regen|→STAGE3 */
#endif /* !LOCK_STAGE2 */
    } else if (s_if_cruise_phase == 3u) {
        outer_cruise_iq_min_slew_apply(ctx, dt);
#if M1_IF_OBS_CRUISE_STEP_ENABLE
        outer_cruise_step_tick(ctx, dt);
#endif
#if M1_IF_OBS_CRUISE_S3_PROBE_ENABLE
        /* 探针未开时保持 249；240/241/242 由 probe 覆盖 */
        if (s_s3_probe_phase == 0u) {
            dbg.open_seq_phase = EXP_MARK_CRUISE_STAGE3_WAIT;
        }
        outer_cruise_s3_probe_tick(ctx, dt);
#else
        dbg.open_seq_phase = EXP_MARK_CRUISE_STAGE3_WAIT;
#endif
#if M1_IF_OBS_CRUISE_LOCK_STAGE3_ENABLE
        (void)omega_fb;
        return;
#else
        if (outer_cruise_iq_auth_ready() == 0u) {
            return;
        }
        if (s_if_cruise_phase_s >= M1_IF_OBS_CRUISE_STAGE2_S) {
            ctx->pi_speed.kp = M1_IF_OBS_CRUISE_PI_KP;
            ctx->pi_speed.ki = M1_IF_OBS_CRUISE_PI_KI;
            s_cruise_iq_min_cmd = -M1_IF_OBS_CRUISE_IQ_ABS_MAX;
            s_cruise_iq_min_target = -M1_IF_OBS_CRUISE_IQ_ABS_MAX;
            s_cruise_iq_max_cmd = M1_IF_OBS_CRUISE_IQ_ABS_MAX;
            s_cruise_iq_max_target = M1_IF_OBS_CRUISE_IQ_ABS_MAX;
            outer_cruise_apply_iq_minmax(ctx,
                                         s_cruise_iq_min_cmd,
                                         s_cruise_iq_max_cmd);
            outer_cruise_bumpless(ctx, omega_fb);
            s_if_cruise_phase = 4u;
            s_if_cruise_phase_s = 0.0f;
            dbg.open_seq_phase = EXP_MARK_CRUISE_PI; /* 正常 PI */
        }
#endif
    }
}

#if M1_IF_OBS_DIR_SEQ_ENABLE
/**
 * @brief 1=滑行段，速度环应松手，Iq/Id 强制 0。
 */
uint8_t motor_outer_if_obs_dir_seq_is_coast(void)
{
    return s_dir_seq_coast;
}

uint8_t motor_outer_if_obs_dir_seq_consume_obs_reset(void)
{
    if (s_dir_seq_obs_reset_req == 0u) {
        return 0u;
    }
    s_dir_seq_obs_reset_req = 0u;
    return 1u;
}

/**
 * @brief 近零后请求再起反向 I-f。读一次清一次。
 */
uint8_t motor_outer_if_obs_dir_seq_consume_rearm(void)
{
    if (s_dir_seq_rearm_req == 0u) {
        return 0u;
    }
    s_dir_seq_rearm_req = 0u;
    s_dir_seq_coast = 0u;
    s_if_cruise_phase = 0u;
    s_if_cruise_phase_s = 0.0f;
    s_dir_seq_leg = 1u; /* 下一腿反转 */
    return 1u;
}
#endif
#endif

#if M1_POS_LOOP_ENABLE && M1_OUTER_NEST_ENABLE && \
    ((M1_OUTER_EXPT == M1_OUTER_EXPT_POS_STEP) || \
     (M1_OUTER_EXPT == M1_OUTER_EXPT_POS_REV) || \
     (M1_OUTER_EXPT == M1_OUTER_EXPT_MIT_HOLD) || \
     (M1_OUTER_EXPT == M1_OUTER_EXPT_MIT_REV))
static uint8_t s_pos_mini_armed;
static uint8_t s_pos_mini_need_th0;
static uint8_t s_pos_mini_done;
static uint8_t s_pos_mini_seq;
static uint32_t s_pos_mini_tick;
static uint32_t s_pos_mini_hold;
static float s_pos_mini_th0;

#if (M1_OUTER_EXPT == M1_OUTER_EXPT_POS_REV) || \
    (M1_OUTER_EXPT == M1_OUTER_EXPT_MIT_REV)
static const float s_pos_mini_amp_rad = 90.0f * 0.01745329252f;
#if M1_OUTER_EXPT == M1_OUTER_EXPT_MIT_REV
static const uint8_t s_pos_mini_phase0 = 230u;
#else
static const uint8_t s_pos_mini_phase0 = 210u;
#endif
#elif M1_OUTER_EXPT == M1_OUTER_EXPT_MIT_HOLD
static const uint8_t s_pos_mini_phase0 = 220u;
#else
static const float s_pos_mini_amp_rad = 10.0f * 0.01745329252f;
static const uint8_t s_pos_mini_phase0 = 200u;
#endif

/**
 * @brief 从当前机械角起表。STEP/REV 走阶跃；MIT_HOLD 钉住；MIT_REV 同 REV。跑完 Iq=0。
 */
void motor_outer_pos_mini_arm(motor_context_t *ctx)
{
    (void)ctx;
    s_pos_mini_armed = 1u;
    s_pos_mini_need_th0 = 1u;
    s_pos_mini_done = 0u;
    s_pos_mini_seq = 0u;
    s_pos_mini_tick = 0u;
    s_pos_mini_hold = 0u;
    s_pos_mini_th0 = 0.0f;
}

/**
 * @brief 外环 DISABLE 且 Iq=0，速度 PI 清零。
 */
static void motor_outer_pos_mini_stop(motor_context_t *ctx)
{
    s_pos_mini_done = 1u;
    s_pos_mini_armed = 0u;
    ctx->theta_ref_rad = s_pos_mini_th0;
    ctx->iq_ref = 0.0f;
    ctx->id_ref = 0.0f;
    ctx->omega_ref = 0.0f;
    foc_pi_reset(&ctx->pi_speed);
    ctx->outer_mode = M1_OUTER_DISABLED;
    dbg.outer_mode = (uint8_t)M1_OUTER_DISABLED;
    dbg.open_seq_phase = EXP_MARK_SIGN_DONE;
    dbg.outer_profile_step = EXP_MARK_SIGN_DONE;
}

/**
 * @brief 推进位置时间表。第一拍用实测角钉 θ0。
 */
static void motor_outer_pos_mini_tick(motor_context_t *ctx, float theta_mech_rad)
{
#if M1_OUTER_EXPT != M1_OUTER_EXPT_MIT_HOLD
    const uint32_t hold_1s =
        (uint32_t)(1.0f / M1_SPEED_TS_S + 0.5f);
    const uint32_t dwell_1_5s =
        (uint32_t)(1.5f / M1_SPEED_TS_S + 0.5f);
    uint8_t apply = 0u;
#endif

    if ((ctx == NULL) || (s_pos_mini_armed == 0u) || (s_pos_mini_done != 0u)) {
        return;
    }

    if (s_pos_mini_need_th0 != 0u) {
        s_pos_mini_th0 = theta_mech_rad;
        ctx->theta_ref_rad = theta_mech_rad;
#if M1_OUTER_EXPT == M1_OUTER_EXPT_MIT_HOLD
        s_pos_mini_hold = (uint32_t)(8.0f / M1_SPEED_TS_S + 0.5f);
#else
        s_pos_mini_hold = (hold_1s == 0u) ? 1u : hold_1s;
#endif
        if (s_pos_mini_hold == 0u) {
            s_pos_mini_hold = 1u;
        }
        s_pos_mini_tick = 0u;
        s_pos_mini_need_th0 = 0u;
        dbg.open_seq_phase = s_pos_mini_phase0;
        dbg.outer_profile_step = 0u;
        return;
    }

    s_pos_mini_tick++;
    if (s_pos_mini_tick < s_pos_mini_hold) {
        return;
    }

#if M1_OUTER_EXPT == M1_OUTER_EXPT_MIT_HOLD
    motor_outer_pos_mini_stop(ctx);
    return;
#else
    s_pos_mini_tick = 0u;
    s_pos_mini_seq++;
#if (M1_OUTER_EXPT == M1_OUTER_EXPT_POS_REV) || \
    (M1_OUTER_EXPT == M1_OUTER_EXPT_MIT_REV)
    if (s_pos_mini_seq <= 5u) {
        if ((s_pos_mini_seq & 1u) != 0u) {
            ctx->theta_ref_rad = s_pos_mini_th0 + s_pos_mini_amp_rad;
        } else {
            ctx->theta_ref_rad = s_pos_mini_th0 - s_pos_mini_amp_rad;
        }
        apply = 1u;
    } else if (s_pos_mini_seq == 6u) {
        ctx->theta_ref_rad = s_pos_mini_th0;
        apply = 1u;
    }
#else
    if (s_pos_mini_seq == 1u) {
        ctx->theta_ref_rad = s_pos_mini_th0 + s_pos_mini_amp_rad;
        apply = 1u;
    } else if (s_pos_mini_seq == 2u) {
        ctx->theta_ref_rad = s_pos_mini_th0;
        apply = 1u;
    } else if (s_pos_mini_seq == 3u) {
        ctx->theta_ref_rad = s_pos_mini_th0 - s_pos_mini_amp_rad;
        apply = 1u;
    } else if (s_pos_mini_seq == 4u) {
        ctx->theta_ref_rad = s_pos_mini_th0;
        apply = 1u;
    }
#endif
    if (apply == 0u) {
        motor_outer_pos_mini_stop(ctx);
        return;
    }

    s_pos_mini_hold = (dwell_1_5s == 0u) ? 1u : dwell_1_5s;
    dbg.open_seq_phase = (uint8_t)(s_pos_mini_phase0 + s_pos_mini_seq);
    dbg.outer_profile_step = s_pos_mini_seq;
#endif
}
#endif

#if M1_POS_LOOP_ENABLE && M1_OUTER_NEST_ENABLE
/* ---- 签收档：一条上电时间表跑完位置环与 MIT 的验收项 ---- */

/** 段号。一次上电按 0..8 顺序跑，遥测 seq 按段打点。 */
#define OUTER_SIGN_SEG_STEP   0u   /* 位置环阶跃幅度扫 */
#define OUTER_SIGN_SEG_SINE   1u   /* 位置环小信号扫频 */
#define OUTER_SIGN_SEG_TRAP   2u   /* 位置环梯形轨迹 */
#define OUTER_SIGN_SEG_DIST   3u   /* 位置环扰动恢复（iq_ff 注入） */
#define OUTER_SIGN_SEG_PREV   4u   /* 位置环正反切 */
#define OUTER_SIGN_SEG_MREV   5u   /* MIT 正反切 */
#define OUTER_SIGN_SEG_MK     6u   /* MIT 刚度标定（iq_ff 注入） */
#define OUTER_SIGN_SEG_MSINE  7u   /* MIT 小信号扫频 */
#define OUTER_SIGN_SEG_REL    8u   /* MIT 推-放（自动松手，测回收） */
#define OUTER_SIGN_SEG_COUNT  9u
/** 全表跑完后的段号，只用于遥测。 */
#define OUTER_SIGN_SEG_DONE   0xFFu

/** 每段开头回 θ0 稳定的时间，单位 s。 */
#define OUTER_SIGN_SETTLE_S   0.8f

/** 超速守卫上电宽限期，单位 s。PLL 首帧冲顶不得算超速。 */
#if (M1_OUTER_THETA_FB_SRC == 2)
/** 无感：等 HFI 锁相+踢段后再计守卫宽限。 */
#define OUTER_GUARD_GRACE_S   12.0f
#else
#define OUTER_GUARD_GRACE_S   1.5f
#endif
/** 超速守卫需连续超限的节拍数，10 ms。 */
#define OUTER_GUARD_HOLD_TICKS  20u

#if M1_OUTER_SIGNOFF_PACK1
#define OUTER_SIGN_STEP_DWELL_S  2.0f
#else
#define OUTER_SIGN_STEP_DWELL_S  1.5f
#endif
#define OUTER_SIGN_TRAP_DWELL_S  1.5f
#define OUTER_SIGN_FF_DWELL_S    2.5f
/** 位置环扰动档的注入脉冲宽度，单位 s。缩短版 0.3→0.5，否则 1 A 档只推出 0.2–0.5°。 */
#define OUTER_SIGN_FF_PULSE_S    0.5f
#define OUTER_SIGN_REV_DWELL_S   1.5f
/** 推-放档：每档先推 2 s，再自动松手；回收实测 <0.1 s，松手段缩短版 3→1.5 s。 */
#define OUTER_SIGN_REL_DWELL_S   3.5f
#define OUTER_SIGN_REL_PUSH_S    2.0f
/** 位置环正弦幅度，单位 rad。取 3°：摩擦抖动约 0.15°，太小会被摩擦污染 Bode。 */
#define OUTER_SIGN_SINE_AMP_RAD  (3.0f * 0.01745329252f)
/**
 * MIT 正弦幅度，单位 rad。取 15°。
 * 静摩擦 0.43 A 折算成偏转是 0.43/3.25=7.6°，2° 的驱动力只有 0.11 A，转子根本不动，
 * 测到的只是残余抖动；15° 给出 0.85 A≈2 倍摩擦，才量得到真实柔度。
 */
#define OUTER_SIGN_MSINE_AMP_RAD (15.0f * 0.01745329252f)

#if !M1_OUTER_SIGNOFF_PACK2
/**
 * 角度表单位 deg。阶跃扫每档夹一个 0 档：先回 θ0 再走 ±A，
 * 否则相邻反号档连在一起，实际阶跃变成 2A，ts/Mp 的幅度就对不上。
 * 缩短版：±1/±5/±30/±90。PACK1 只钉 θ0 保持 2 s，用来确认位置环没叠前馈。
 */
#if M1_OUTER_SIGNOFF_PACK1
static const float s_sign_step_deg[] = {
    0.0f
};
#else
static const float s_sign_step_deg[] = {
    1.0f, 0.0f, -1.0f, 0.0f,
    5.0f, 0.0f, -5.0f, 0.0f,
    30.0f, 0.0f, -30.0f, 0.0f,
    90.0f, 0.0f, -90.0f, 0.0f
};
#endif
#endif
static const float s_sign_sine_hz[] = {
    0.5f, 1.0f, 2.0f, 4.0f, 8.0f
};
static const float s_sign_trap_deg[] = {
    90.0f, -90.0f, 30.0f, -30.0f, 10.0f, -10.0f
};
/**
 * 电流表单位 A，供段4 扰动、段7 刚度、段9 推-放共用。
 * 缩短版：±0.5 档推不动（低于/接近静摩擦 0.43 A）被删，改 ±1/±3 A，四档。
 * 段4 脉宽也由 0.3 s 加到 0.5 s，否则 1 A 档只推出 0.2–0.5°。
 */
#if (M1_OUTER_THETA_FB_SRC == 2) && M1_OUTER_SIGNOFF_PACK1
/** 无感冒烟推-放：只用 ±1 A，避免 3 A 顶飞观测。 */
static const float s_sign_ff_a[] = {
    1.0f, -1.0f
};
#else
static const float s_sign_ff_a[] = {
    1.0f, -1.0f, 3.0f, -3.0f
};
#endif
#if M1_OUTER_SIGNOFF_PACK1
#if (M1_OUTER_THETA_FB_SRC == 2)
/** 无感冒烟：±30° 正反切，少推大角。 */
static const float s_sign_rev_deg[] = {
    30.0f, -30.0f
};
#else
static const float s_sign_rev_deg[] = {
    90.0f, -90.0f
};
#endif
#else
static const float s_sign_rev_deg[] = {
    90.0f, -90.0f, 90.0f, -90.0f, 90.0f, 0.0f
};
#endif

/**
 * 段起始 seq。8 位可容纳，段内再叠档位号。
 * 段1 有 28 档（含回零），所以扫频段改到 45 起，与段1 的 10..37 不重叠。
 * PREV 段仍用 210、MREV 段仍用 230，与旧 POS_REV、MIT_REV 的录波对齐。
 */
static const uint8_t s_sign_seq_base[OUTER_SIGN_SEG_COUNT] = {
    10u, 45u, 60u, 70u, 210u, 230u, 80u, 95u, 150u
};

/**
 * 梯形轨迹状态。plan 求分段时长，eval 按已走时间出位置/速度/加速度。
 * 单位：位置 rad，速度 rad/s，加速度 rad/s^2，时间 s。
 */
typedef struct {
    float ar;
    float dr;
    float vr;
    float ta;
    float tv;
    float td;
    float tf;
    float xi;
    float xf;
    float vi;
    float y_accel;
    float t;
} outer_trap_t;

static uint8_t s_sign_armed;
static uint8_t s_sign_need_th0;
static uint8_t s_sign_seg;
static uint8_t s_sign_idx;
static uint32_t s_sign_t;
static float s_sign_th0;
static outer_trap_t s_sign_trap;
#if M1_OUTER_SIGNOFF_PACK2
/** 按机械角 2π 均匀分的保持电流表，单位 A。 */
static float s_fric_map[M1_FRIC_MAP_N];
/** 1=爬表完成，后续出口叠加查表。 */
static uint8_t s_fric_map_valid;
static float s_map_acc;
static uint32_t s_map_cnt;
static float s_map_last_th;
#endif

/**
 * @brief 取符号，0 归正。
 * @param v 输入。
 * @return +1 或 -1。
 */
static float outer_sign_sign(float v)
{
    return (v < 0.0f) ? -1.0f : 1.0f;
}

/**
 * @brief 秒换算成外环节拍数。
 * @param s 时间，单位 s。
 * @return 节拍数，至少 1。
 */
static uint32_t outer_sign_ticks(float s)
{
    uint32_t n = (uint32_t)(s / M1_SPEED_TS_S + 0.5f);

    return (n == 0u) ? 1u : n;
}

#if M1_OUTER_SIGNOFF_PACK2
/**
 * @brief 机械角折到 [0, 2π)。
 * @param th 机械角，单位 rad。
 * @return 同单位。
 */
static float outer_wrap_0_2pi(float th)
{
    const float twopi = 6.28318530718f;

    th = fmodf(th, twopi);
    if (th < 0.0f) {
        th += twopi;
    }
    return th;
}

/**
 * @brief 把积分器平均写入摩擦表对应格。
 * @param th_rad 该点目标机械角，单位 rad。
 */
static void outer_fric_map_flush_th(float th_rad)
{
    const float twopi = 6.28318530718f;
    uint32_t i;

    if (s_map_cnt == 0u) {
        return;
    }
    i = (uint32_t)(outer_wrap_0_2pi(th_rad) *
                   (float)M1_FRIC_MAP_N / twopi);
    if (i >= M1_FRIC_MAP_N) {
        i = M1_FRIC_MAP_N - 1u;
    }
    s_fric_map[i] = s_map_acc / (float)s_map_cnt;
}

/**
 * @brief 按机械角线性插值摩擦表。
 * @param th_rad 反馈机械角，单位 rad。
 * @return 前馈电流，单位 A。表未就绪时为 0。
 */
static float outer_fric_map_lookup(float th_rad)
{
    const float twopi = 6.28318530718f;
    const float n = (float)M1_FRIC_MAP_N;
    float u;
    float f;
    uint32_t i0;
    uint32_t i1;

    if (s_fric_map_valid == 0u) {
        return 0.0f;
    }
    u = outer_wrap_0_2pi(th_rad) * n / twopi;
    i0 = (uint32_t)u;
    f = u - (float)i0;
    if (i0 >= M1_FRIC_MAP_N) {
        i0 = 0u;
    }
    i1 = i0 + 1u;
    if (i1 >= M1_FRIC_MAP_N) {
        i1 = 0u;
    }
    return s_fric_map[i0] * (1.0f - f) + s_fric_map[i1] * f;
}
#endif

/**
 * @brief 规划梯形轨迹，求加/减/巡航分段时长。
 * @param tr 轨迹状态。不可为 NULL。
 * @param xf 终点，单位 rad。
 * @param xi 起点，单位 rad。
 * @param vi 起点速度，单位 rad/s。
 * @param vmax 巡航速度上限，单位 rad/s。
 * @param amax 加速度上限，单位 rad/s^2。
 * @param dmax 减速度上限，单位 rad/s^2。
 * @note 三个上限任一非正时退化为直接到点，不产生 NaN。
 */
static void outer_trap_plan(outer_trap_t *tr, float xf, float xi, float vi,
                            float vmax, float amax, float dmax)
{
    float dx;
    float stop_dist;
    float dx_stop;
    float s;
    float dx_min;

    tr->xi = xi;
    tr->xf = xf;
    tr->vi = vi;
    tr->t = 0.0f;
    tr->ar = 0.0f;
    tr->dr = 0.0f;
    tr->vr = 0.0f;
    tr->ta = 0.0f;
    tr->tv = 0.0f;
    tr->td = 0.0f;
    tr->tf = 0.0f;
    tr->y_accel = xi;

    if ((vmax <= 0.0f) || (amax <= 0.0f) || (dmax <= 0.0f)) {
        return;
    }

    dx = xf - xi;
    stop_dist = (vi * vi) / (2.0f * dmax);
    dx_stop = (vi < 0.0f) ? -stop_dist : stop_dist;
    s = outer_sign_sign(dx - dx_stop);
    tr->ar = s * amax;
    tr->dr = -s * dmax;
    tr->vr = s * vmax;

    if ((s * vi) > (s * tr->vr)) {
        tr->ar = -s * amax;
    }

    tr->ta = (tr->vr - vi) / tr->ar;
    tr->td = -tr->vr / tr->dr;
    dx_min = 0.5f * tr->ta * (tr->vr + vi) + 0.5f * tr->td * tr->vr;

    if ((s * dx) < (s * dx_min)) {
        const float den = tr->dr - tr->ar;
        float q = 0.0f;

        if (den != 0.0f) {
            q = (tr->dr * vi * vi + 2.0f * tr->ar * tr->dr * dx) / den;
        }
        if (q < 0.0f) {
            q = 0.0f;
        }
        tr->vr = s * sqrtf(q);
        tr->ta = (tr->vr - vi) / tr->ar;
        if (tr->ta < 0.0f) {
            tr->ta = 0.0f;
        }
        tr->td = -tr->vr / tr->dr;
        if (tr->td < 0.0f) {
            tr->td = 0.0f;
        }
        tr->tv = 0.0f;
    } else {
        tr->tv = (dx - dx_min) / tr->vr;
        if (tr->tv < 0.0f) {
            tr->tv = 0.0f;
        }
    }

    tr->tf = tr->ta + tr->tv + tr->td;
    tr->y_accel = xi + vi * tr->ta + 0.5f * tr->ar * tr->ta * tr->ta;
}

/**
 * @brief 按已走时间出轨迹位置/速度/加速度，并把内部时间推进一拍。
 * @param tr 轨迹状态。不可为 NULL。
 * @param dt 节拍，单位 s。
 * @param y 位置输出，单位 rad。
 * @param yd 速度输出，单位 rad/s。
 * @param ydd 加速度输出，单位 rad/s^2。
 */
static void outer_trap_eval(outer_trap_t *tr, float dt,
                            float *y, float *yd, float *ydd)
{
    const float t = tr->t;

    if (t < tr->ta) {
        *y = tr->xi + tr->vi * t + 0.5f * tr->ar * t * t;
        *yd = tr->vi + tr->ar * t;
        *ydd = tr->ar;
    } else if (t < (tr->ta + tr->tv)) {
        *y = tr->y_accel + tr->vr * (t - tr->ta);
        *yd = tr->vr;
        *ydd = 0.0f;
    } else if (t < tr->tf) {
        const float t_neg = t - tr->tf;

        *y = tr->xf + 0.5f * tr->dr * t_neg * t_neg;
        *yd = tr->dr * t_neg;
        *ydd = tr->dr;
    } else {
        *y = tr->xf;
        *yd = 0.0f;
        *ydd = 0.0f;
    }

    tr->t += dt;
}

/**
 * @brief 用梯形轨迹走一个目标角，写 θ_ref 与速度前馈。
 * @param ctx 控制上下文。不可为 NULL。
 * @param target_rad 目标机械角，单位 rad。
 * @param idx 档位号，变化时重新规划。
 * @note 起点取 ctx->theta_ref_rad（上一档的设定值），不能取段起点 s_sign_th0。
 *       取 s_sign_th0 会让新档第一拍把 θ* 从上一档目标跳回 θ0，产生几十度的假跟踪误差。
 *       plan 在本函数里先于 eval 调用，此处 theta_ref 仍是上一拍写的值。
 */
static void outer_sign_trap_move(motor_context_t *ctx, float target_rad,
                                 uint8_t idx)
{
    float y;
    float yd;
    float ydd;

    if (idx != s_sign_idx) {
        s_sign_idx = idx;
        outer_trap_plan(&s_sign_trap, target_rad, ctx->theta_ref_rad, 0.0f,
                        M1_OUTER_TRAJ_VMAX_RPM * 0.10471975512f,
                        M1_OUTER_TRAJ_AMAX_RAD_S2,
                        M1_OUTER_TRAJ_DMAX_RAD_S2);
    }

    outer_trap_eval(&s_sign_trap, M1_SPEED_TS_S, &y, &yd, &ydd);
    ctx->theta_ref_rad = y;
    ctx->traj_vel_rpm = yd * 9.54929658551f;
    ctx->iq_ff = 0.0f;
}

#if M1_FRIC_FF_ENABLE
/** 库仑前馈的分段开关。1=允许（运行类段），0=禁止（标定/线性诊断段）。 */
static uint8_t s_fric_ff_en = 1u;

/**
 * @brief 库仑前馈：A * sat(θ_err / δ)。ω≈0 时 sign(ω) 为 0，只能用位置误差。
 * @param err_rad 位置误差 θ_ref−θ_fb，单位 rad。
 * @return 前馈电流，单位 A，幅值不超过 M1_FRIC_FF_A。
 * @note 连续饱和，不用符号滞回。带内多一段刚度 A/δ，带外才满幅。
 */
static float outer_fric_ff_step(float err_rad)
{
    float u;

    if (s_fric_ff_en == 0u) {
        return 0.0f;
    }
    if (M1_FRIC_FF_SAT_RAD <= 0.0f) {
        return 0.0f;
    }

    u = err_rad / M1_FRIC_FF_SAT_RAD;
    if (u > 1.0f) {
        u = 1.0f;
    } else if (u < -1.0f) {
        u = -1.0f;
    }
    return M1_FRIC_FF_A * u;
}
#endif /* M1_FRIC_FF_ENABLE */

/**
 * @brief 进段：播种到 θ0、选外环模式、清积分与激励。
 * @param ctx 控制上下文。不可为 NULL。
 * @param seg 目标段号。
 * @param omega_rpm 当前机械转速，单位 rpm，用于无扰切换。
 */
static void outer_sign_enter(motor_context_t *ctx, uint8_t seg, float omega_rpm)
{
    s_sign_seg = seg;
    /* 置无效值，保证本段第一拍一定重新规划轨迹 */
    s_sign_idx = 0xFFu;
    s_sign_t = 0u;
    ctx->iq_ff = 0.0f;
    ctx->traj_vel_rpm = 0.0f;
    ctx->theta_ref_rad = s_sign_th0;

    if ((seg == OUTER_SIGN_SEG_MREV) || (seg == OUTER_SIGN_SEG_MK) ||
        (seg == OUTER_SIGN_SEG_MSINE) || (seg == OUTER_SIGN_SEG_REL)) {
        motor_outer_set_mode(ctx, M1_OUTER_MIT, ctx->iq, omega_rpm);
    } else {
        motor_outer_set_mode(ctx, M1_OUTER_POSITION, ctx->iq, omega_rpm);
    }

#if M1_FRIC_FF_ENABLE
    /*
     * 饱和前馈只留 MIT 正反切与推-放。位置环开着积分，再叠会跟积分抢。
     * 爬表完成后改走 τ(θ)，饱和关掉。
     */
    s_fric_ff_en = 0u;
    if (((seg == OUTER_SIGN_SEG_MREV) || (seg == OUTER_SIGN_SEG_REL))
#if M1_OUTER_SIGNOFF_PACK2
        && (s_fric_map_valid == 0u)
#endif
        ) {
        s_fric_ff_en = 1u;
    }
#endif
    foc_pi_reset(&ctx->pi_speed);
    dbg.outer_sign_seg = seg;
    dbg.outer_sign_sub = 0u;
}

/**
 * @brief 停表：seq 打点。跑完与守卫共用。
 * @param ctx 控制上下文。不可为 NULL。
 * @param seq 打点号。255=正常跑完，252=超速守卫。
 * @note 无感 θ_fb 且正常跑完时保持位置环钉在 θ0，避免表一结束就松手像崩了。
 *       守卫停仍关断外环。
 */
static void outer_sign_stop(motor_context_t *ctx, uint8_t seq)
{
    s_sign_armed = 0u;
    s_sign_seg = OUTER_SIGN_SEG_DONE;
    s_sign_idx = 0u;
    ctx->theta_ref_rad = s_sign_th0;
    ctx->iq_ff = 0.0f;
    ctx->traj_vel_rpm = 0.0f;
    ctx->id_ref = 0.0f;
    ctx->omega_ref = 0.0f;
#if (M1_OUTER_THETA_FB_SRC == 2)
    if (seq == EXP_MARK_SIGN_DONE) {
        foc_pi_reset(&ctx->pi_speed);
        motor_outer_set_mode(ctx, M1_OUTER_POSITION, ctx->iq, 0.0f);
        ctx->theta_ref_rad = s_sign_th0;
    } else {
        ctx->iq_ref = 0.0f;
        ctx->outer_mode = M1_OUTER_DISABLED;
        dbg.outer_mode = (uint8_t)M1_OUTER_DISABLED;
        foc_pi_reset(&ctx->pi_speed);
    }
#else
    ctx->iq_ref = 0.0f;
    ctx->outer_mode = M1_OUTER_DISABLED;
    dbg.outer_mode = (uint8_t)M1_OUTER_DISABLED;
    foc_pi_reset(&ctx->pi_speed);
#endif
    dbg.outer_sign_seg = OUTER_SIGN_SEG_DONE;
    dbg.outer_sign_sub = 0u;
    dbg.open_seq_phase = seq;
    dbg.outer_profile_step = EXP_MARK_SIGN_DONE;
}

/**
 * @brief 武装签收时间表，段 0 段首回 θ0。只从上电路径调用。
 * @param ctx 控制上下文。不可为 NULL。
 */
void motor_outer_signoff_arm(motor_context_t *ctx)
{
    (void)ctx;
    s_sign_armed = 1u;
    s_sign_need_th0 = 1u;
    s_sign_seg = 0u;
    s_sign_idx = 0u;
    s_sign_t = 0u;
    s_sign_th0 = 0.0f;
    dbg.outer_sign_seg = 0u;
    dbg.outer_sign_sub = 0u;
#if M1_OUTER_SIGNOFF_PACK2
    {
        uint32_t i;

        for (i = 0u; i < M1_FRIC_MAP_N; i++) {
            s_fric_map[i] = 0.0f;
        }
        s_fric_map_valid = 0u;
        s_map_acc = 0.0f;
        s_map_cnt = 0u;
        s_map_last_th = 0.0f;
    }
#endif
}

/**
 * @brief 签收时间表是否已武装。
 * @return 1=已武装，0=未武装。
 */
uint8_t motor_outer_signoff_is_armed(void)
{
    return s_sign_armed;
}


/**
 * @brief 正弦扫频段按累计时长定位当前频点。
 * @param rel 段内相对节拍，不含段首稳段。
 * @param idx 输出频点号。
 * @param t_in 输出该频点内已走节拍。
 * @return 1=仍在表内，0=表已走完。
 */
static uint8_t outer_sign_sine_locate(uint32_t rel, uint8_t *idx,
                                     uint32_t *t_in)
{
    const uint8_t n =
        (uint8_t)(sizeof(s_sign_sine_hz) / sizeof(s_sign_sine_hz[0]));
    uint32_t acc = 0u;
    uint8_t i;

    for (i = 0u; i < n; i++) {
        const float cyc = 4.0f / s_sign_sine_hz[i];   /* 缩短版：每点 5→4 周期 */
        const float dur = (cyc > 0.5f) ? cyc : 0.5f;
        const uint32_t d = outer_sign_ticks(dur);

        if (rel < (acc + d)) {
            *idx = i;
            *t_in = rel - acc;
            return 1u;
        }
        acc += d;
    }
    return 0u;
}

/**
 * @brief 超速守卫：|ω| 超 ω_max×容差即停环并打点。
 * @param ctx 控制上下文。不可为 NULL。
 * @param omega_rpm 当前机械转速，单位 rpm。
 * @return 1=已停环，本拍不要再算。
 * @note 上电有宽限期。有感时与编码器裸差分交叉核对，避免 PLL 冲顶误跳；
 *       无感 θ_fb 时不做交叉核对，只信观测转速。
 */
static uint8_t outer_pos_overspeed_guard(motor_context_t *ctx, float omega_rpm)
{
    const float lim = M1_OUTER_VEL_LIMIT_TOLERANCE * M1_POS_OMEGA_MAX_RPM;
    static uint32_t s_guard_boot_ticks;
    static uint8_t s_guard_viol_n;

    s_guard_boot_ticks++;
    if (s_guard_boot_ticks < outer_sign_ticks(OUTER_GUARD_GRACE_S)) {
        return 0u;
    }

    if ((omega_rpm <= lim) && (omega_rpm >= -lim)) {
        s_guard_viol_n = 0u;
        return 0u;
    }

#if (M1_OUTER_THETA_FB_SRC != 2)
    if ((dbg.pll_omega_diff_rpm <= (lim * 0.5f)) &&
        (dbg.pll_omega_diff_rpm >= (-lim * 0.5f))) {
        /* PLL 说超速但编码器裸差分没动：判定为观测冲顶，不跳 */
        s_guard_viol_n = 0u;
        return 0u;
    }
#endif

    if (s_guard_viol_n < 250u) {
        s_guard_viol_n++;
    }
    if (s_guard_viol_n < OUTER_GUARD_HOLD_TICKS) {
        return 0u;
    }

    if (s_sign_armed != 0u) {
        outer_sign_stop(ctx, EXP_MARK_SIGN_GUARD);
        return 1u;
    }

    ctx->iq_ref = 0.0f;
    ctx->iq_ff = 0.0f;
    ctx->traj_vel_rpm = 0.0f;
    ctx->id_ref = 0.0f;
    ctx->omega_ref = 0.0f;
    ctx->outer_mode = M1_OUTER_DISABLED;
    dbg.outer_mode = (uint8_t)M1_OUTER_DISABLED;
    foc_pi_reset(&ctx->pi_speed);
    dbg.open_seq_phase = EXP_MARK_SIGN_GUARD;
    return 1u;
}


/**
 * @brief PACK1 是否保留该段。
 * @param seg 段号。
 * @return 1=跑，0=跳过。
 */
static uint8_t outer_sign_seg_keep(uint8_t seg)
{
#if (M1_OUTER_SIGNOFF_PACK1 || M1_OUTER_SIGNOFF_PACK2)
    return ((seg == OUTER_SIGN_SEG_STEP) ||
            (seg == OUTER_SIGN_SEG_MREV) ||
            (seg == OUTER_SIGN_SEG_REL)) ? 1u : 0u;
#else
    (void)seg;
    return 1u;
#endif
}

/**
 * @brief 进下一段；已是最后一段则停环。
 * @param ctx 控制上下文。不可为 NULL。
 * @param omega_rpm 当前机械转速，单位 rpm。
 */
static void outer_sign_advance(motor_context_t *ctx, float omega_rpm)
{
    uint8_t next = (uint8_t)(s_sign_seg + 1u);

    while ((next < OUTER_SIGN_SEG_COUNT) &&
           (outer_sign_seg_keep(next) == 0u)) {
        next++;
    }
    if (next >= OUTER_SIGN_SEG_COUNT) {
        outer_sign_stop(ctx, EXP_MARK_SIGN_DONE);
        return;
    }
    outer_sign_enter(ctx, next, omega_rpm);
}

/**
 * @brief 段 0：位置环阶跃幅度扫。PACK2 时改为绕一圈爬摩擦表。
 * @param ctx 控制上下文。不可为 NULL。
 * @param rel 段内相对节拍。
 * @param omega_rpm 当前机械转速，单位 rpm。
 */
static void outer_sign_seg_step(motor_context_t *ctx, uint32_t rel,
                                float omega_rpm)
{
#if M1_OUTER_SIGNOFF_PACK2
    const uint32_t dwell = outer_sign_ticks(M1_FRIC_MAP_DWELL_S);
    const uint32_t samp = outer_sign_ticks(M1_FRIC_MAP_SAMP_S);
    const uint8_t n = (uint8_t)M1_FRIC_MAP_N;
    const uint8_t idx = (uint8_t)(rel / dwell);
    const uint32_t t_in = rel % dwell;
    const float twopi = 6.28318530718f;

    if (idx >= n) {
        outer_fric_map_flush_th(s_map_last_th);
        s_fric_map_valid = 1u;
        outer_sign_advance(ctx, omega_rpm);
        return;
    }
    if (idx != s_sign_idx) {
        if (s_sign_idx != 0xFFu) {
            outer_fric_map_flush_th(s_map_last_th);
        }
        s_map_acc = 0.0f;
        s_map_cnt = 0u;
        s_map_last_th = s_sign_th0 + twopi * (float)idx / (float)n;
    }
    outer_sign_trap_move(ctx, s_map_last_th, idx);
    if ((t_in + samp) >= dwell) {
        s_map_acc += ctx->pi_speed.integrator;
        s_map_cnt++;
    }
#else
    const uint8_t n =
        (uint8_t)(sizeof(s_sign_step_deg) / sizeof(s_sign_step_deg[0]));
    const uint32_t dwell = outer_sign_ticks(OUTER_SIGN_STEP_DWELL_S);
    const uint8_t idx = (uint8_t)(rel / dwell);

    if (idx >= n) {
        outer_sign_advance(ctx, omega_rpm);
        return;
    }

    ctx->theta_ref_rad = s_sign_th0 + s_sign_step_deg[idx] * 0.01745329252f;
    ctx->traj_vel_rpm = 0.0f;
    ctx->iq_ff = 0.0f;
    s_sign_idx = idx;
#endif
}

/**
 * @brief 正弦扫频段：叠 θ0 上做小信号。
 * @param ctx 控制上下文。不可为 NULL。
 * @param rel 段内相对节拍。
 * @param amp_rad 正弦幅度，单位 rad。
 * @param omega_rpm 当前机械转速，单位 rpm。
 * @note 刻意不加速度前馈：带前馈时位置靠前馈跟住，测出的是"前馈跟踪"而不是环路带宽。
 */
static void outer_sign_seg_sine(motor_context_t *ctx, uint32_t rel,
                                float amp_rad, float omega_rpm)
{
    uint8_t idx = 0u;
    uint32_t t_in = 0u;
    float f;
    float ph;

    if (outer_sign_sine_locate(rel, &idx, &t_in) == 0u) {
        outer_sign_advance(ctx, omega_rpm);
        return;
    }

    f = s_sign_sine_hz[idx];
    ph = 6.28318530718f * f * ((float)t_in * M1_SPEED_TS_S);
    ctx->theta_ref_rad = s_sign_th0 + amp_rad * sinf(ph);
    ctx->traj_vel_rpm = 0.0f;
    ctx->iq_ff = 0.0f;
    s_sign_idx = idx;
}

/**
 * @brief 梯形轨迹段：按角度表逐个到位，档间靠 dwell 分。
 * @param ctx 控制上下文。不可为 NULL。
 * @param rel 段内相对节拍。
 * @param tab 目标角表，单位 deg。
 * @param n 表长。
 * @param omega_rpm 当前机械转速，单位 rpm。
 */
static void outer_sign_seg_trap(motor_context_t *ctx, uint32_t rel,
                                const float *tab, uint8_t n, float omega_rpm)
{
    const uint32_t dwell = outer_sign_ticks(OUTER_SIGN_TRAP_DWELL_S);
    const uint8_t idx = (uint8_t)(rel / dwell);

    if (idx >= n) {
        outer_sign_advance(ctx, omega_rpm);
        return;
    }
    outer_sign_trap_move(ctx, s_sign_th0 + tab[idx] * 0.01745329252f, idx);
}

/**
 * @brief 电流前馈段：θ* 钉 θ0，逐档加 iq_ff。
 * @param ctx 控制上下文。不可为 NULL。
 * @param rel 段内相对节拍。
 * @param omega_rpm 当前机械转速，单位 rpm。
 * @param pulse 1=只加窄脉冲（位置环扰动），0=整档常值（MIT 刚度）。
 * @note 位置环带积分，常值力矩只会等速漂移，所以扰动档必须用脉冲。
 */
static void outer_sign_seg_ff(motor_context_t *ctx, uint32_t rel,
                              float omega_rpm, uint8_t pulse)
{
    const uint8_t n =
        (uint8_t)(sizeof(s_sign_ff_a) / sizeof(s_sign_ff_a[0]));
    const uint32_t dwell = outer_sign_ticks(OUTER_SIGN_FF_DWELL_S);
    const uint8_t idx = (uint8_t)(rel / dwell);

    if (idx >= n) {
        outer_sign_advance(ctx, omega_rpm);
        return;
    }

    ctx->theta_ref_rad = s_sign_th0;
    ctx->traj_vel_rpm = 0.0f;
    if (pulse != 0u) {
        ctx->iq_ff = ((rel % dwell) < outer_sign_ticks(OUTER_SIGN_FF_PULSE_S)) ?
                     s_sign_ff_a[idx] : 0.0f;
    } else {
        ctx->iq_ff = s_sign_ff_a[idx];
    }
    s_sign_idx = idx;
}

/**
 * @brief 推-放段：θ* 钉 θ0，先加已知 iq_ff 推到位，再自动松手。
 * @param ctx 控制上下文。不可为 NULL。
 * @param rel 段内相对节拍。
 * @param omega_rpm 当前机械转速，单位 rpm。
 * @note 松手段就是撤掉已知外力，等价于阻抗的阶跃响应；比手拨可重复且有已知力值。
 */
static void outer_sign_seg_rel(motor_context_t *ctx, uint32_t rel,
                               float omega_rpm)
{
    const uint8_t n =
        (uint8_t)(sizeof(s_sign_ff_a) / sizeof(s_sign_ff_a[0]));
    const uint32_t dwell = outer_sign_ticks(OUTER_SIGN_REL_DWELL_S);
    const uint32_t push = outer_sign_ticks(OUTER_SIGN_REL_PUSH_S);
    const uint8_t idx = (uint8_t)(rel / dwell);
    const uint32_t t_in = rel % dwell;

    if (idx >= n) {
        outer_sign_advance(ctx, omega_rpm);
        return;
    }

    ctx->theta_ref_rad = s_sign_th0;
    ctx->traj_vel_rpm = 0.0f;
    ctx->iq_ff = (t_in < push) ? s_sign_ff_a[idx] : 0.0f;
    s_sign_idx = idx;
}

/**
 * @brief 推进签收时间表。段首先回 θ0 稳 OUTER_SIGN_SETTLE_S。
 * @param ctx 控制上下文。不可为 NULL。
 * @param theta_fb 本拍位置反馈角，单位 rad。
 * @param omega_rpm 本拍机械转速，单位 rpm。
 * @note 第一拍只播种 θ0 并进段 0，不出力。E0 经 exp_runner SCRIPT 调用本函数。
 */
void motor_outer_signoff_tick(motor_context_t *ctx,
                              float theta_fb,
                              float omega_rpm)
{
    uint32_t sett;
    uint32_t rel;

    if ((ctx == NULL) || (s_sign_armed == 0u)) {
        return;
    }

    if (s_sign_need_th0 != 0u) {
        s_sign_th0 = theta_fb;
        s_sign_need_th0 = 0u;
        outer_sign_enter(ctx, OUTER_SIGN_SEG_STEP, omega_rpm);
        return;
    }

    if (s_sign_seg >= OUTER_SIGN_SEG_COUNT) {
        return;
    }

    s_sign_t++;
    sett = outer_sign_ticks(OUTER_SIGN_SETTLE_S);
    if (s_sign_t <= sett) {
        /* 段首：回 θ0 稳定，清掉上一段的激励 */
        ctx->theta_ref_rad = s_sign_th0;
        ctx->traj_vel_rpm = 0.0f;
        ctx->iq_ff = 0.0f;
        return;
    }
    rel = s_sign_t - sett;

    switch (s_sign_seg) {
    case OUTER_SIGN_SEG_STEP:
        outer_sign_seg_step(ctx, rel, omega_rpm);
        break;
    case OUTER_SIGN_SEG_SINE:
        outer_sign_seg_sine(ctx, rel, OUTER_SIGN_SINE_AMP_RAD, omega_rpm);
        break;
    case OUTER_SIGN_SEG_TRAP:
        outer_sign_seg_trap(ctx, rel, s_sign_trap_deg,
                            (uint8_t)(sizeof(s_sign_trap_deg) /
                                      sizeof(s_sign_trap_deg[0])),
                            omega_rpm);
        break;
    case OUTER_SIGN_SEG_DIST:
        /* 位置环：扰动用脉冲，常值会让位置等速漂移 */
        outer_sign_seg_ff(ctx, rel, omega_rpm, 1u);
        break;
    case OUTER_SIGN_SEG_PREV:
        outer_sign_seg_trap(ctx, rel, s_sign_rev_deg,
                            (uint8_t)(sizeof(s_sign_rev_deg) /
                                      sizeof(s_sign_rev_deg[0])),
                            omega_rpm);
        break;
    case OUTER_SIGN_SEG_MREV:
        outer_sign_seg_trap(ctx, rel, s_sign_rev_deg,
                            (uint8_t)(sizeof(s_sign_rev_deg) /
                                      sizeof(s_sign_rev_deg[0])),
                            omega_rpm);
        break;
    case OUTER_SIGN_SEG_MK:
        /* MIT：整档常值砝码，Ki 关时有有限静差 */
        outer_sign_seg_ff(ctx, rel, omega_rpm, 0u);
        break;
    case OUTER_SIGN_SEG_MSINE:
        outer_sign_seg_sine(ctx, rel, OUTER_SIGN_MSINE_AMP_RAD, omega_rpm);
        break;
    case OUTER_SIGN_SEG_REL:
        outer_sign_seg_rel(ctx, rel, omega_rpm);
        break;
    default:
        break;
    }

    if (s_sign_seg < OUTER_SIGN_SEG_COUNT) {
        dbg.outer_sign_seg = s_sign_seg;
        dbg.outer_sign_sub = (s_sign_idx == 0xFFu) ? 0u : s_sign_idx;
        dbg.open_seq_phase =
            (uint8_t)(s_sign_seq_base[s_sign_seg] + dbg.outer_sign_sub);
    }
}

#endif /* M1_POS_LOOP_ENABLE && M1_OUTER_NEST_ENABLE */




#if M1_POS_LOOP_ENABLE && M1_OUTER_NEST_ENABLE
/**
 * @brief 位置 P 再进速度 PI。MIT 档叠标定倍率，速度前馈来自轨迹。
 * @param ctx 控制上下文。不可为 NULL。
 * @param theta_mech_rad 位置反馈角，单位 rad。
 * @param omega_mech_rpm 机械转速，单位 rpm。
 * @param mit_ki_hold 1=MIT 档：速度积分冻结；0=位置档：积分正常。
 * @note 电流前馈 iq_ff 在 PI 之后叠加再限幅，供刚度标定与扰动实验用。
 */
static void outer_nested_pos_mit_step(motor_context_t *ctx,
                                      float theta_mech_rad,
                                      float omega_mech_rpm,
                                      uint8_t mit_ki_hold)
{
    const float err_rad = ctx->theta_ref_rad - theta_mech_rad;
    float omega_cmd_rpm;

    if (mit_ki_hold != 0u) {
        omega_cmd_rpm = ctx->traj_vel_rpm +
                        (M1_POS_KP_RPM_PER_RAD * M1_MIT_KP_SCALE) * err_rad -
                        M1_MIT_KD_RPM * omega_mech_rpm;
#if M1_MIT_KI_DISABLE
        ctx->pi_speed.integrator = 0.0f;
#endif
    } else {
        omega_cmd_rpm = ctx->traj_vel_rpm +
                        M1_POS_KP_RPM_PER_RAD * err_rad;
    }

    ctx->omega_ref = outer_clamp_omega_ref(omega_cmd_rpm);
    ctx->iq_ref = outer_speed_pi_step(ctx, ctx->omega_ref, omega_mech_rpm,
                                      mit_ki_hold);
#if M1_FRIC_FF_ENABLE
    if ((ctx->atrb & M1_ATRB_FRIC_FF) != 0u) {
        ctx->iq_fric_ff = outer_fric_ff_step(err_rad);
    } else {
        ctx->iq_fric_ff = 0.0f;
    }
#else
    ctx->iq_fric_ff = 0.0f;
#endif
#if M1_OUTER_SIGNOFF_PACK2
    if ((ctx->atrb & M1_ATRB_FRIC_FF) != 0u) {
        ctx->iq_fric_ff += outer_fric_map_lookup(theta_mech_rad);
    }
#endif
    if ((ctx->iq_ff != 0.0f) || (ctx->iq_fric_ff != 0.0f)) {
        ctx->iq_ref = outer_clamp_iq_ref(ctx,
                                         ctx->iq_ref + ctx->iq_ff +
                                         ctx->iq_fric_ff);
    }
    ctx->id_ref = 0.0f;
}
#endif

/**
 * @brief 按外环模式写 id_ref / iq_ref。DISABLED 不写。
 * @param ctx 控制上下文。不可为 NULL。
 */
void motor_outer_loop_tick(motor_context_t *ctx)
{
    float omega_mech_rpm;
    float theta_mech_rad;
    float theta_fb_rad;

    if (ctx == NULL) {
        return;
    }

#if M1_ID_LOCK_CAL_SWEEP
    /* Pass0 / VASI：外环不得写 iq_ref/ω_ref（旧版 speed 联调曾导致转子旋转） */
    if (deadband_flow_id_cal_active()) {
        return;
    }
#endif

#if M1_PLL_ENABLE
    omega_mech_rpm = motor_current_get_pll_omega_mech_rpm();
#else
    omega_mech_rpm = 0.0f;
#endif
    theta_mech_rad = motor_current_get_theta_mech_rad();
    theta_fb_rad = motor_current_get_theta_fb_rad();

#if M1_POS_LOOP_ENABLE && M1_OUTER_NEST_ENABLE
    if ((ctx->outer_mode == M1_OUTER_POSITION) ||
        (ctx->outer_mode == M1_OUTER_MIT)) {
        /* 先推进时间表（含 θ0 播种），再判守卫：守卫停环时 θ* 已有正确基准 */
#if M1_EXP_FRAMEWORK_ENABLE
        exp_runner_tick(ctx, theta_fb_rad, omega_mech_rpm);
#else
        motor_outer_signoff_tick(ctx, theta_fb_rad, omega_mech_rpm);
#endif
#if M1_OUTER_OVERSPEED_GUARD_ENABLE
        if (outer_pos_overspeed_guard(ctx, omega_mech_rpm) != 0u) {
            return;
        }
#endif
    }
#endif

    switch (ctx->outer_mode) {
    case M1_OUTER_TORQUE:
        ctx->iq_ref = outer_clamp_iq_ref(ctx, ctx->iq_cmd);
        ctx->id_ref = 0.0f;
        break;

#if M1_POS_LOOP_ENABLE
    case M1_OUTER_POSITION:
#if M1_OUTER_NEST_ENABLE
#if (M1_OUTER_EXPT == M1_OUTER_EXPT_POS_STEP) || \
    (M1_OUTER_EXPT == M1_OUTER_EXPT_POS_REV)
        motor_outer_pos_mini_tick(ctx, theta_fb_rad);
        if (s_pos_mini_done != 0u) {
            break;
        }
#endif
        outer_nested_pos_mit_step(ctx, theta_fb_rad, omega_mech_rpm, 0u);
        break;
    case M1_OUTER_MIT:
#if (M1_OUTER_EXPT == M1_OUTER_EXPT_MIT_HOLD) || \
    (M1_OUTER_EXPT == M1_OUTER_EXPT_MIT_REV)
        motor_outer_pos_mini_tick(ctx, theta_fb_rad);
        if (s_pos_mini_done != 0u) {
            break;
        }
#endif
        outer_nested_pos_mit_step(ctx, theta_fb_rad, omega_mech_rpm, 1u);
        break;
#else
#if M1_POS_STEP_TEST_ENABLE
        if (s_pos_step_armed) {
                    motor_pos_step_test_tick(ctx);
            }
#endif
            {
            uint8_t pos_p_tick = 0u;

#if M1_POS_DECIM > 1u
            s_pos_p_div++;
            if (s_pos_p_div >= M1_POS_DECIM) {
                s_pos_p_div = 0u;
                pos_p_tick = 1u;
            }
#else
            pos_p_tick = 1u;
#endif

            if (pos_p_tick != 0u) {
                const float err_rad = ctx->theta_ref_rad - theta_mech_rad;

#if M1_POS_ERR_HYST_ENABLE
                const uint8_t was_hold = s_pos_err_hold_zone;

                ctx->omega_ref = outer_pos_omega_from_err(err_rad);
#if M1_POS_ERR_HYST_FREEZE_SPEED_PI
                if (s_pos_err_hold_zone) {
                    if (was_hold == 0u) {
                        foc_pi_reset(&ctx->pi_speed);
                    }
                } else if (was_hold != 0u) {
                    foc_pi_bumpless_beta(&ctx->pi_speed, 0.0f,
                                         ctx->omega_ref, omega_mech_rpm,
                                         M1_SPEED_PI_BETA);
                }
#else
                (void)was_hold;
#endif
#else
                ctx->omega_ref =
                    outer_clamp_omega_ref(M1_POS_KP_RPM_PER_RAD * err_rad);
#endif /* M1_POS_ERR_HYST_ENABLE */
            }

#if M1_POS_ERR_HYST_ENABLE && M1_POS_ERR_HYST_FREEZE_SPEED_PI
            if (s_pos_err_hold_zone) {
                ctx->iq_ref = 0.0f;
            } else
#endif
            {
#if M1_SPEED_OMEGA_RAMP_ENABLE
                ctx->iq_ref = outer_speed_pi_step(ctx,
                                                   outer_omega_ref_ramped(ctx),
                                                   omega_mech_rpm,
                                                   0u);
#else
                ctx->iq_ref = outer_speed_pi_step(ctx,
                                                   ctx->omega_ref,
                                                   omega_mech_rpm,
                                                   0u);
#endif
            }
        }
        ctx->id_ref = 0.0f;
        break;
#endif /* !M1_OUTER_NEST_ENABLE */
#endif /* M1_POS_LOOP_ENABLE */

    case M1_OUTER_SPEED:
        {
            uint8_t speed_hold_i = 0u;
            uint8_t force_iq_zero = 0u;

#if M1_SPEED_IDENT_ENABLE
            if (speed_ident_flow_is_armed()) {
                speed_ident_flow_tick(ctx);
            } else
#endif
            {
#if M1_SPEED_REVERSAL_TEST_ENABLE
                if (s_reversal_armed) {
                    motor_speed_reversal_tick(ctx);
                }
#endif
#if M1_SPEED_PROFILE_ENABLE
#if M1_SPEED_REVERSAL_TEST_ENABLE
                else if (!s_reversal_armed)
#else
                if (1)
#endif
                {
                    motor_speed_profile_tick(ctx);
                }
#endif
            }
#if M1_SPEED_IDENT_ENABLE
            if (speed_ident_flow_is_armed() &&
                speed_ident_module_hold_iq_inhibit()) {
                speed_hold_i = 1u;
                force_iq_zero = 1u;
            }
#endif
            /* 电压饱和：冻积分，避免 iq_ref→11A 而电流环顶满（2030） */
            {
                const float ud = dbg.foc_ud_out;
                const float uq = dbg.foc_uq_out;
                const float um2 = ud * ud + uq * uq;
                const float lim = 0.95f * M1_PI_V_MAX;

                if (um2 >= (lim * lim)) {
                    speed_hold_i = 1u;
                }
            }
#if M1_SPEED_OMEGA_RAMP_ENABLE
            ctx->iq_ref = outer_speed_pi_step(ctx,
                                               outer_omega_ref_ramped(ctx),
                                               omega_mech_rpm,
                                               speed_hold_i);
#else
            ctx->iq_ref = outer_speed_pi_step(ctx,
                                               ctx->omega_ref,
                                               omega_mech_rpm,
                                               speed_hold_i);
#endif
            if (force_iq_zero != 0u) {
                ctx->iq_ref = 0.0f;
            }
            ctx->id_ref = 0.0f;
        }
        break;

    case M1_OUTER_DISABLED:
    default:
        /* 不写 iq_ref：保持 startup / 标定 / API 写入的值 */
        break;
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
#if M1_POS_ERR_HYST_ENABLE
    outer_pos_err_hyst_reset();
#endif
#if M1_POS_DECIM > 1u
    outer_pos_p_decim_arm();
#endif
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

#if M1_SPEED_OMEGA_RAMP_ENABLE
    s_omega_ramped_rpm = omega_now;
#endif
#if M1_SPEED_IQ_SLEW_ENABLE
    s_iq_ref_slew = iq_meas;
#endif
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
#if M1_SPEED_OMEGA_RAMP_ENABLE
    s_omega_ramped_rpm = omega_rpm;
#else
    (void)omega_rpm;
#endif
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
        /* TORQUE/DISABLED/POSITION → SPEED：用当前 iq_meas 做 bumpless */
        foc_pi_bumpless_beta(&ctx->pi_speed, iq_meas,
                              ctx->omega_ref, omega_now,
                              M1_SPEED_PI_BETA);
#if M1_SPEED_OMEGA_RAMP_ENABLE
        s_omega_ramped_rpm = omega_now;
#endif
#if M1_SPEED_IQ_SLEW_ENABLE
        s_iq_ref_slew = iq_meas;
#endif
        break;

#if M1_POS_LOOP_ENABLE
    case M1_OUTER_POSITION:
            motor_outer_arm_position_hold(ctx);
#if M1_POS_ERR_HYST_ENABLE
            outer_pos_err_hyst_reset();
#endif
#if M1_POS_DECIM > 1u
            outer_pos_p_decim_arm();
#endif
        foc_pi_bumpless_beta(&ctx->pi_speed, iq_meas, 0.0f, omega_now,
                              M1_SPEED_PI_BETA);
#if M1_SPEED_OMEGA_RAMP_ENABLE
        s_omega_ramped_rpm = omega_now;
#endif
#if M1_SPEED_IQ_SLEW_ENABLE
        s_iq_ref_slew = iq_meas;
#endif
        break;
#if M1_OUTER_NEST_ENABLE
    case M1_OUTER_MIT:
        motor_outer_arm_position_hold(ctx);
        foc_pi_reset(&ctx->pi_speed);
        break;
#endif
#endif

    case M1_OUTER_TORQUE:
        /* SPEED → TORQUE：重置速度 PI，防止下次切回时积分残留 */
        foc_pi_reset(&ctx->pi_speed);
        break;

    case M1_OUTER_DISABLED:
    default:
        /* 切回 DISABLED 时重置，重新进入 startup/标定不残留速度 PI 积分 */
        foc_pi_reset(&ctx->pi_speed);
        break;
    }

    ctx->outer_mode = new_mode;
    dbg.outer_mode = (uint8_t)new_mode;
}

#else /* M1_SPEED_LOOP_ENABLE */

void motor_outer_loop_init(motor_context_t *ctx)
{
    (void)ctx;
}

void motor_outer_loop_tick(motor_context_t *ctx)
{
    (void)ctx;
}

void motor_outer_set_mode(motor_context_t *ctx,
                           m1_outer_mode_t new_mode,
                           float iq_meas,
                           float omega_now)
{
    (void)ctx;
    (void)new_mode;
    (void)iq_meas;
    (void)omega_now;
}

void motor_outer_arm_position_hold(motor_context_t *ctx)
{
    (void)ctx;
}

#endif /* M1_SPEED_LOOP_ENABLE */
