/**
 * @file defaults_ident_iq.h
 * @date 2026-10-28
 * @brief Iq 堵转辨识默认宏（仅 M1_IDENT_ENABLE=1 时由 motor_params 纳入）。
 */
#ifndef CONFIG_DEFAULTS_IDENT_IQ_H
#define CONFIG_DEFAULTS_IDENT_IQ_H

#if M1_ID_LOCK_CAL_SWEEP && !M1_IDENT_ID_CAL_BEFORE_STEP
#error "M1_IDENT_ENABLE requires M1_ID_LOCK_CAL_SWEEP=0 (or M1_IDENT_ID_CAL_BEFORE_STEP=1)"
#endif

#ifndef M1_IDENT_ID_CAL_BEFORE_STEP
#define M1_IDENT_ID_CAL_BEFORE_STEP   0
#endif

#ifndef M1_IDENT_IQ_STEP_ENABLE
#define M1_IDENT_IQ_STEP_ENABLE     1
#endif
#ifndef M1_IDENT_IQ_BODE_ENABLE
#define M1_IDENT_IQ_BODE_ENABLE     0   /* 先阶跃；=1 。STEP 后接 BODE */
#endif
#if !M1_IDENT_IQ_STEP_ENABLE && !M1_IDENT_IQ_BODE_ENABLE
#error "M1_IDENT_ENABLE requires M1_IDENT_IQ_STEP_ENABLE and/or M1_IDENT_IQ_BODE_ENABLE"
#endif


/** 制动器加载后稳定等待 (s) */
#ifndef M1_IDENT_HOLD_S
#define M1_IDENT_HOLD_S               2.0f
#endif
/** 阶跃总轮。/ OFF / FIXED：IDENT 模式在上。#elif 已设。18 / 6 / 6；此处仅为未覆盖时的兜底 */
#ifndef M1_IDENT_STEP_ROUNDS
#define M1_IDENT_STEP_ROUNDS          18u
#endif
#ifndef M1_IDENT_STEP_OFF_ROUNDS
#define M1_IDENT_STEP_OFF_ROUNDS      6u
#endif
#ifndef M1_IDENT_STEP_FIXED_ROUNDS
#define M1_IDENT_STEP_FIXED_ROUNDS    6u
#endif
/** 1 。FIXED_ROUNDS=0：OFF 之后全用 LUT；FIXED_ROUNDS>0 时第三段恒为 LUT */
#if M1_IDENT_IQ_STEP_ENABLE
#if (M1_IDENT_STEP_OFF_ROUNDS > M1_IDENT_STEP_ROUNDS)
#error "M1_IDENT_STEP_OFF_ROUNDS must be <= M1_IDENT_STEP_ROUNDS"
#endif
#if (M1_IDENT_STEP_OFF_ROUNDS + M1_IDENT_STEP_FIXED_ROUNDS > M1_IDENT_STEP_ROUNDS)
#error "M1_IDENT_STEP_OFF_ROUNDS + M1_IDENT_STEP_FIXED_ROUNDS must be <= M1_IDENT_STEP_ROUNDS"
#endif
#if ((M1_IDENT_STEP_ROUNDS - M1_IDENT_STEP_OFF_ROUNDS - M1_IDENT_STEP_FIXED_ROUNDS) > 0u) && \
    !M1_DEADBAND_LUT_BAKED_ENABLE && !M1_IDENT_ID_CAL_BEFORE_STEP
#error "LUT segment requires M1_DEADBAND_LUT_BAKED_ENABLE or M1_IDENT_ID_CAL_BEFORE_STEP"
#endif
#endif /* M1_IDENT_IQ_STEP_ENABLE */
#ifndef M1_IDENT_STEP_I0_A
#define M1_IDENT_STEP_I0_A            0.0f
#endif
#ifndef M1_IDENT_STEP_I1_A
#define M1_IDENT_STEP_I1_A            0.3f   /* 小阶。*/
#endif
#ifndef M1_IDENT_STEP_I2_A
#define M1_IDENT_STEP_I2_A            0.5f   /* 中阶。*/
#endif
#ifndef M1_IDENT_STEP_I3_A
#define M1_IDENT_STEP_I3_A            1.0f   /* 低段大阶。*/
#endif
#ifndef M1_IDENT_STEP_I_BASE_HI_A
#define M1_IDENT_STEP_I_BASE_HI_A     1.0f   /* 高段 1 A 基线 */
#endif
#ifndef M1_IDENT_STEP_I5_A
#define M1_IDENT_STEP_I5_A            1.3f
#endif
#ifndef M1_IDENT_STEP_I6_A
#define M1_IDENT_STEP_I6_A            1.5f
#endif
#ifndef M1_IDENT_STEP_I7_A
#define M1_IDENT_STEP_I7_A            2.0f
#endif
#ifndef M1_IDENT_STEP_BANDS
#define M1_IDENT_STEP_BANDS           1u
#endif
#ifndef M1_IDENT_STEP_ROUNDS_PER_PROFILE
#define M1_IDENT_STEP_ROUNDS_PER_PROFILE  M1_IDENT_STEP_OFF_ROUNDS
#endif
#if (M1_IDENT_STEP_BANDS > 1u) && \
    (M1_IDENT_STEP_ROUNDS != (M1_IDENT_STEP_BANDS * 2u * M1_IDENT_STEP_ROUNDS_PER_PROFILE))
#error "M1_IDENT_STEP_ROUNDS must equal M1_IDENT_STEP_BANDS * 2 * M1_IDENT_STEP_ROUNDS_PER_PROFILE"
#endif
/** 各拍 dwell (s)；非零档与回零档相同 */
#ifndef M1_IDENT_STEP_DWELL_S
#define M1_IDENT_STEP_DWELL_S         0.5f
#endif
#ifndef M1_IDENT_STEP_ZERO_DWELL_S
#define M1_IDENT_STEP_ZERO_DWELL_S    0.5f
#endif

/** 1=阶跃辨识绕过 M1_CLOSURE_BRINGUP 。I_ref 钳位。6 V PI 限幅 */
#ifndef M1_IDENT_OVERRIDE_LIMITS
#define M1_IDENT_OVERRIDE_LIMITS      1
#endif
#if M1_IDENT_OVERRIDE_LIMITS
#define M1_IDENT_PI_V_LIMIT_V         M1_PI_V_MAX
#else
#define M1_IDENT_PI_V_LIMIT_V         M1_PI_V_LIMIT_V
#endif
#define M1_IDENT_PI_V_LIMIT_MIN       (-M1_IDENT_PI_V_LIMIT_V)
#define M1_IDENT_PI_INT_LIMIT_V       M1_IDENT_PI_V_LIMIT_V
#define M1_IDENT_PI_INT_LIMIT_MIN     (-M1_IDENT_PI_V_LIMIT_V)

/** Bode 轴：0=Iq sin（Id=0）；1=Id sin（Iq=0，。锁轴。*/
#ifndef M1_IDENT_BODE_AXIS_ID
#define M1_IDENT_BODE_AXIS_ID         0
#endif

/** Bode：i_ref = bias + amp*sin(2πft)，几何扫。f *= RATIO */
#ifndef M1_IDENT_BODE_I_BIAS_A
#define M1_IDENT_BODE_I_BIAS_A        0.25f
#endif
#ifndef M1_IDENT_BODE_I_AMP_A
#define M1_IDENT_BODE_I_AMP_A         0.05f
#endif
#ifndef M1_IDENT_BODE_I_BIAS_HI_A
#define M1_IDENT_BODE_I_BIAS_HI_A     1.25f   /* 。I Bode；无 1.0 A 。*/
#endif
#ifndef M1_IDENT_BODE_I_AMP_HI_A
#define M1_IDENT_BODE_I_AMP_HI_A      0.10f
#endif
#ifndef M1_IDENT_BODE_LUT_ENABLE
#define M1_IDENT_BODE_LUT_ENABLE      1
#endif
#ifndef M1_IDENT_BODE_BANDS
#define M1_IDENT_BODE_BANDS           1u
#endif
#if M1_IDENT_BODE_LUT_ENABLE
#if (M1_IDENT_BODE_BANDS > 1u) && (M1_IDENT_BODE_ROUNDS != (M1_IDENT_BODE_BANDS * 2u))
#error "M1_IDENT_BODE_ROUNDS must equal M1_IDENT_BODE_BANDS * 2"
#endif
#else
#if (M1_IDENT_BODE_BANDS > 1u) && (M1_IDENT_BODE_ROUNDS != M1_IDENT_BODE_BANDS)
#error "OFF-only Bode: M1_IDENT_BODE_ROUNDS must equal M1_IDENT_BODE_BANDS"
#endif
#endif
#ifndef M1_IDENT_BODE_F_SPLIT_HZ
#define M1_IDENT_BODE_F_SPLIT_HZ      500.0f
#endif
#ifndef M1_IDENT_BODE_CYCLES_PER_FREQ
#define M1_IDENT_BODE_CYCLES_PER_FREQ 20.0f
#endif
/** f >= F_SPLIT 固定观测窗：1=Ncyc=T_obs*f。=。CYCLES_HI 常数 */
#ifndef M1_IDENT_BODE_USE_T_OBS_HI
#define M1_IDENT_BODE_USE_T_OBS_HI    1
#endif
#ifndef M1_IDENT_BODE_T_OBS_HI_S
#define M1_IDENT_BODE_T_OBS_HI_S      0.1f   /* 100 ms；@1 kHz 。100 cycles */
#endif
#ifndef M1_IDENT_BODE_CYCLES_HI
#define M1_IDENT_BODE_CYCLES_HI       50.0f  /* 。USE_T_OBS_HI=0 时生。*/
#endif
/** Bode 分段。×OFF + 1×FIXED + 1×LUT，各。F0→F1 一整遍 */
#ifndef M1_IDENT_BODE_ROUNDS
#define M1_IDENT_BODE_ROUNDS          3u
#endif
#ifndef M1_IDENT_BODE_OFF_ROUNDS
#define M1_IDENT_BODE_OFF_ROUNDS      1u
#endif
#ifndef M1_IDENT_BODE_FIXED_ROUNDS
#define M1_IDENT_BODE_FIXED_ROUNDS    1u
#endif
#if M1_IDENT_IQ_BODE_ENABLE && M1_IDENT_BODE_LUT_ENABLE
#if (M1_IDENT_BODE_OFF_ROUNDS + M1_IDENT_BODE_FIXED_ROUNDS > M1_IDENT_BODE_ROUNDS)
#error "M1_IDENT_BODE_OFF_ROUNDS + M1_IDENT_BODE_FIXED_ROUNDS must be <= M1_IDENT_BODE_ROUNDS"
#endif
#if ((M1_IDENT_BODE_ROUNDS - M1_IDENT_BODE_OFF_ROUNDS - M1_IDENT_BODE_FIXED_ROUNDS) > 0u) && \
    !M1_DEADBAND_LUT_BAKED_ENABLE && !M1_IDENT_ID_CAL_BEFORE_STEP
#error "Bode LUT segment requires M1_IDENT_ID_CAL_BEFORE_STEP or M1_DEADBAND_LUT_BAKED_ENABLE"
#endif
#endif /* M1_IDENT_IQ_BODE_ENABLE && M1_IDENT_BODE_LUT_ENABLE */

#endif /* CONFIG_DEFAULTS_IDENT_IQ_H */