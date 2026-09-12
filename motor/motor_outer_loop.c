/**
 * @file motor_outer_loop.c
 * @brief M1 2kHz 外环调度实现：DISABLED / TORQUE / SPEED / POSITION + bumpless 切换。
 */

#include "motor_outer_loop.h"

#include <math.h>
#include <stddef.h>

#include "dbg_monitor.h"
#include "deadband_flow.h"
#include "foc_pi.h"
#include "motor_current.h"
#include "motor_if.h"
#include "motor_params_m1.h"
#include "speed_ident_flow.h"
#if M1_SPEED_IDENT_ENABLE
#include "speed_ident_module.h"
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
    dbg.outer_omega_ref = ctx->omega_ref;
    dbg.outer_profile_step = 0u;
}

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
    dbg.outer_omega_ref = ctx->omega_ref;
    dbg.outer_profile_step = s_profile_step;
}
#endif /* M1_SPEED_PROFILE_ENABLE */

#if M1_SPEED_REVERSAL_TEST_ENABLE
static uint8_t s_reversal_armed;
static uint32_t s_reversal_tick;
static int8_t s_reversal_sign;

void motor_speed_reversal_arm(motor_context_t *ctx)
{
    if (ctx == NULL) {
        return;
    }

    s_reversal_armed = 1u;
    s_reversal_tick = 0u;
    s_reversal_sign = 1;
    ctx->omega_ref = M1_SPEED_REVERSAL_RPM;
    dbg.outer_omega_ref = ctx->omega_ref;
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
    dbg.outer_omega_ref = ctx->omega_ref;
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
    dbg.outer_theta_ref_rad = ctx->theta_ref_rad;
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
    dbg.outer_theta_ref_rad = ctx->theta_ref_rad;
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
    const float step = M1_SPEED_OMEGA_RAMP_RPM_S * M1_SPEED_TS_S;
    float delta = target - s_omega_ramped_rpm;

    if (delta > step) {
        s_omega_ramped_rpm += step;
    } else if (delta < -step) {
        s_omega_ramped_rpm -= step;
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
    dbg.outer_omega_ref = rpm;
#if M1_SPEED_OMEGA_RAMP_ENABLE
    /* 硬阶跃：把斜坡状态钉到目标，否则 200rpm/s 会抹平响应 */
    motor_outer_set_omega_ramp_rpm(rpm);
#endif
    dbg.open_seq_phase = (uint8_t)(250u + idx); /* 250..255 */
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
        dbg.open_seq_phase = 239u; /* 阶跃表结束（249 留给③） */
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
    dbg.outer_omega_ref = rpm;
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
            dbg.open_seq_phase = 249u; /* ③ 到位，等探针 */
            return;
        }
        /* 先拉到 HI，再砸向 LO，形成大减速 */
        outer_cruise_s3_probe_apply(ctx, M1_IF_OBS_CRUISE_STEP_HI_RPM, 240u);
        s_s3_probe_phase = 1u;
        s_s3_probe_s = 0.0f;
        return;
    }

    if (s_s3_probe_s < M1_IF_OBS_CRUISE_S3_PROBE_HOLD_S) {
        return;
    }

    if (s_s3_probe_phase == 1u) {
        outer_cruise_s3_probe_apply(ctx, M1_IF_OBS_CRUISE_STEP_LO_RPM, 241u); /* HI→LO */
        s_s3_probe_phase = 2u;
        s_s3_probe_s = 0.0f;
    } else if (s_s3_probe_phase == 2u) {
        outer_cruise_s3_probe_apply(ctx, M1_IF_OBS_CRUISE_RPM, 242u); /* →基 */
        s_s3_probe_phase = 3u;
        s_s3_probe_s = 0.0f;
    } else if (s_s3_probe_phase == 3u) {
        s_s3_probe_phase = 4u;
        dbg.open_seq_phase = 243u; /* s3 probe done */
    }
}
#endif

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
    dbg.outer_omega_ref = cruise_rpm;
    dbg.open_seq_phase = 246u; /* cruise① soft-brake */
}

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
        dbg.outer_omega_ref = 0.0f;
        dbg.open_seq_phase = 230u;
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
                dbg.open_seq_phase = 231u;
            }
        } else {
            s_dir_seq_zero_s = 0.0f;
        }
        (void)omega_fb;
        return;
    }
    if (s_if_cruise_phase == 6u) {
        ctx->omega_ref = 0.0f;
        dbg.outer_omega_ref = 0.0f;
        dbg.open_seq_phase = 231u;
        return;
    }
    if (s_if_cruise_phase == 8u) {
        dbg.open_seq_phase = 232u;
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
        dbg.open_seq_phase = 246u;

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
                    dbg.outer_omega_ref = 0.0f;
                    dbg.open_seq_phase = 230u;
                    return;
                }
                /* 反转站稳 → 结束 */
                s_if_cruise_phase = 8u;
                s_dir_seq_leg = 2u;
                dbg.open_seq_phase = 232u;
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
        dbg.open_seq_phase = 247u; /* ② 限权斜坡 */
#endif /* !LOCK_STAGE1 */
    } else if (s_if_cruise_phase == 2u) {
        outer_cruise_iq_min_slew_apply(ctx, dt);
#if M1_IF_OBS_CRUISE_STEP_ENABLE
        /* step 未开时保持 247；阶跃中用 250–254 */
        if (s_cruise_step_phase == 0u) {
            dbg.open_seq_phase = 247u;
        }
        outer_cruise_step_tick(ctx, dt);
#else
        dbg.open_seq_phase = 247u;
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
        dbg.open_seq_phase = 249u; /* ③ |regen|→STAGE3 */
#endif /* !LOCK_STAGE2 */
    } else if (s_if_cruise_phase == 3u) {
        outer_cruise_iq_min_slew_apply(ctx, dt);
#if M1_IF_OBS_CRUISE_STEP_ENABLE
        outer_cruise_step_tick(ctx, dt);
#endif
#if M1_IF_OBS_CRUISE_S3_PROBE_ENABLE
        /* 探针未开时保持 249；240/241/242 由 probe 覆盖 */
        if (s_s3_probe_phase == 0u) {
            dbg.open_seq_phase = 249u;
        }
        outer_cruise_s3_probe_tick(ctx, dt);
#else
        dbg.open_seq_phase = 249u;
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
            dbg.open_seq_phase = 248u; /* 正常 PI */
        }
#endif
    }
}

#if M1_IF_OBS_DIR_SEQ_ENABLE
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

void motor_outer_loop_tick(motor_context_t *ctx)
{
    float omega_mech_rpm;
    float theta_mech_rad;

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

    dbg.outer_omega_mech_rpm = omega_mech_rpm;
    dbg.outer_theta_ref_rad = ctx->theta_ref_rad;
    dbg.outer_theta_err_rad = ctx->theta_ref_rad - theta_mech_rad;

    switch (ctx->outer_mode) {
    case M1_OUTER_TORQUE:
        ctx->iq_ref = outer_clamp_iq_ref(ctx, ctx->iq_cmd);
        ctx->id_ref = 0.0f;
        break;

#if M1_POS_LOOP_ENABLE
    case M1_OUTER_POSITION:
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
#endif

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

    dbg.outer_iq_ref = ctx->iq_ref;
    dbg.outer_omega_ref = ctx->omega_ref;
}

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
    dbg.outer_theta_ref_rad = ctx->theta_ref_rad;
    dbg.outer_omega_ref = 0.0f;
}

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

void motor_outer_set_omega_ramp_rpm(float omega_rpm)
{
#if M1_SPEED_OMEGA_RAMP_ENABLE
    s_omega_ramped_rpm = omega_rpm;
#else
    (void)omega_rpm;
#endif
}

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
