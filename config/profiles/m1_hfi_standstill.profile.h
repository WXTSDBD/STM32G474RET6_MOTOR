/**
 * @file m1_hfi_standstill.profile.h
 * @brief HFI standstill delivery profile — GATE 141 only.
 *
 * Switch experiments only via config/bringup_active.h (must keep M1_HFI_GATE=141).
 * Demod / PLL / inject stay in hfi_sqwave.c; this file only lays macros.
 */
#ifndef CONFIG_PROFILES_M1_HFI_STANDSTILL_PROFILE_H
#define CONFIG_PROFILES_M1_HFI_STANDSTILL_PROFILE_H

#ifndef M1_HFI_GATE
#define M1_HFI_GATE                     141
#endif
#if M1_HFI_GATE != 141
#error "M1_HFI_GATE must be 141 (delivery profile)"
#endif

#define M1_ID_LOCK_CAL_SWEEP            0
#define M1_IDENT_ENABLE                 0
#define M1_ID_CAL_ALIGN_ENABLE          0
#define M1_ID_CAL_COMMIT_LUT            0
#define M1_LD_LQ_IDENT_ENABLE           0
#define M1_RS_IDENT_ENABLE              0
#define M1_VOFA_IDENT_DUMP_ENABLE       0

#define M1_SPEED_IDENT_ENABLE           0
#define M1_SPEED_IDENT_STEP_ENABLE      0
#define M1_SPEED_IDENT_BODE_ENABLE      0
#define M1_SPEED_PROFILE_ENABLE         0

#undef M1_SPEED_LOOP_ENABLE
#define M1_SPEED_LOOP_ENABLE            1
#undef M1_SPEED_LOOP_BOOT
#define M1_SPEED_LOOP_BOOT              0
#undef M1_SPEED_OMEGA_RAMP_ENABLE
#define M1_SPEED_OMEGA_RAMP_ENABLE      0

#undef M1_OPEN_UD_PRE_ID_CAL_ENABLE
#define M1_OPEN_UD_PRE_ID_CAL_ENABLE    0
#undef M1_OPEN_UQ_PRE_ID_CAL_ENABLE
#define M1_OPEN_UQ_PRE_ID_CAL_ENABLE    0

#undef M1_STARTUP_ENABLE
#define M1_STARTUP_ENABLE               0

#define M1_DEADBAND_ENABLE              0
#define M1_DEADBAND_LUT_BAKED_ENABLE    0
#define M1_DEADBAND_NVM_ON_BOOT         0

#undef M1_IF_ENABLE
#define M1_IF_ENABLE                    0
#undef M1_IF_TO_OBS_ENABLE
#define M1_IF_TO_OBS_ENABLE             0

#undef M1_ENC_OPTIONAL_ENABLE
#define M1_ENC_OPTIONAL_ENABLE          1

#undef M1_IQ_REF_A
#define M1_IQ_REF_A                     0.0f

#undef M1_HFI_ENABLE
#define M1_HFI_ENABLE                   1
#undef M1_HFI_MOTION_BYPASS_ENABLE
#define M1_HFI_MOTION_BYPASS_ENABLE     1
#undef M1_HFI_IPD_SWEEP_ENABLE
#define M1_HFI_IPD_SWEEP_ENABLE         0
#undef M1_HFI_QKICK_SWEEP_ENABLE
#define M1_HFI_QKICK_SWEEP_ENABLE       0
#undef M1_HFI_QKICK_AFTER_LOCK_ENABLE
#define M1_HFI_QKICK_AFTER_LOCK_ENABLE  0
#undef M1_HFI_DQ_IDENT_ENABLE
#define M1_HFI_DQ_IDENT_ENABLE          0
#undef M1_HFI_QKICK_SPEED_ENABLE
#define M1_HFI_QKICK_SPEED_ENABLE       0
#undef M1_HFI_QKICK_BRAKE_ENABLE
#define M1_HFI_QKICK_BRAKE_ENABLE       0
#undef M1_HFI_QKICK_CRAWL_ENABLE
#define M1_HFI_QKICK_CRAWL_ENABLE       0
#undef M1_HFI_QKICK_START_ENABLE
#define M1_HFI_QKICK_START_ENABLE       0
#undef M1_HFI_SENSED_CAL_ENABLE
#define M1_HFI_SENSED_CAL_ENABLE        0

#undef M1_HFI_PLL_EPS_DEAD
#define M1_HFI_PLL_EPS_DEAD             0.0f
#undef M1_HFI_VH_V
#define M1_HFI_VH_V                     0.40f
#undef M1_HFI_BIAS_CAL_ENABLE
#define M1_HFI_BIAS_CAL_ENABLE          0
#undef M1_HFI_SEQ_ENABLE
#define M1_HFI_SEQ_ENABLE               1
#undef M1_HFI_FH_HZ
#define M1_HFI_FH_HZ                    10000.0f

#undef M1_HFI_POLARITY_ENC_ENABLE
#define M1_HFI_POLARITY_ENC_ENABLE      0
#undef M1_HFI_POLARITY_IPD_ENABLE
#define M1_HFI_POLARITY_IPD_ENABLE      0
#undef M1_HFI_INIT_FROM_ENC
#define M1_HFI_INIT_FROM_ENC            0
#undef M1_HFI_PLL_INIT_OFF_RAD
#define M1_HFI_PLL_INIT_OFF_RAD         0.0f

#undef M1_HFI_PLL_ENABLE
#define M1_HFI_PLL_ENABLE               1
#undef M1_HFI_AXIS_SEL_ENABLE
#define M1_HFI_AXIS_SEL_ENABLE          0
#undef M1_HFI_PLL_KP
#define M1_HFI_PLL_KP                   39.2f
#undef M1_HFI_PLL_KI
#define M1_HFI_PLL_KI                   389.0f
#undef M1_HFI_PLL_W_MAX
#define M1_HFI_PLL_W_MAX                200.0f
#undef M1_HFI_PLL_INT_MAX
#define M1_HFI_PLL_INT_MAX              M1_HFI_PLL_W_MAX
#undef M1_HFI_PLL_INT_LEAK
#define M1_HFI_PLL_INT_LEAK             0.0f
#undef M1_HFI_PLL_HOLD_ENABLE
#define M1_HFI_PLL_HOLD_ENABLE          0
#undef M1_HFI_LOCK_ENABLE
#define M1_HFI_LOCK_ENABLE              0
#undef M1_HFI_SPEED_FB_ENABLE
#define M1_HFI_SPEED_FB_ENABLE          0
#undef M1_HFI_OMEGA_FF_SRC
#define M1_HFI_OMEGA_FF_SRC             0
#undef M1_HFI_OMEGA_FF_FROM_REF
#define M1_HFI_OMEGA_FF_FROM_REF        0
#undef M1_HFI_OMEGA_SEED_ENABLE
#define M1_HFI_OMEGA_SEED_ENABLE        0
#undef M1_HFI_EPS_SIGN
#define M1_HFI_EPS_SIGN                 1.0f
#undef M1_HFI_ATAN2_ENABLE
#define M1_HFI_ATAN2_ENABLE             1
#undef M1_HFI_A_CMD
#define M1_HFI_A_CMD                    (0.10f)

#undef M1_EMF_VEQ_ENABLE
#define M1_EMF_VEQ_ENABLE               0
#undef M1_EMF_SMO_ENABLE
#define M1_EMF_SMO_ENABLE               0
#undef M1_EMF_PLL_ENABLE
#define M1_EMF_PLL_ENABLE               0
#undef M1_OBS_SOFT_SWITCH_ENABLE
#define M1_OBS_SOFT_SWITCH_ENABLE       0

#undef M1_PLL_ENABLE
#define M1_PLL_ENABLE                   1
#undef M1_VOFA_UNIFIED_12CH
#define M1_VOFA_UNIFIED_12CH            1
#undef M1_TELEM_BRINGUP_K
#define M1_TELEM_BRINGUP_K              12u
#undef M1_TELEM_BRINGUP_DECIMATION
#define M1_TELEM_BRINGUP_DECIMATION     8u /* 2.5 kHz；原 D=2=10kHz，减遥测 ISR/UART 负载 */

#undef M1_VOFA_HFI_12CH
#define M1_VOFA_HFI_12CH                1
#undef M1_VOFA_IF_12CH
#define M1_VOFA_IF_12CH                 0
#undef M1_VOFA_OBS_SMO_12CH
#define M1_VOFA_OBS_SMO_12CH            0
#undef M1_VOFA_OBS_PLL_12CH
#define M1_VOFA_OBS_PLL_12CH            0
#undef M1_VOFA_OBS_VEQ_12CH
#define M1_VOFA_OBS_VEQ_12CH            0

/* -------------------------------------------------------------------------- */

#if M1_HFI_GATE == 141
/* GATE 141 only: ±1500 rpm hold 0.5 s reverse ×5; pub/inj follow |ω|. */
#undef M1_HFI_DELTA_SWEEP_ENABLE
#define M1_HFI_DELTA_SWEEP_ENABLE       0
#undef M1_HFI_PARK_ENABLE
#define M1_HFI_PARK_ENABLE              1
#undef M1_HFI_BOOT_DELAY_S
#define M1_HFI_BOOT_DELAY_S             5.0f
#undef M1_HFI_RUN_LADDER_ENABLE
#define M1_HFI_RUN_LADDER_ENABLE        1
#undef M1_HFI_RUN_RPM_START
#define M1_HFI_RUN_RPM_START            100.0f
#undef M1_HFI_RUN_RPM_STEP
#define M1_HFI_RUN_RPM_STEP             100.0f
#undef M1_HFI_RUN_RPM_MAX
#define M1_HFI_RUN_RPM_MAX              1500.0f
#undef M1_HFI_RUN_STEP_S
#define M1_HFI_RUN_STEP_S               5.0f
#undef M1_HFI_RUN_RPM1
#define M1_HFI_RUN_RPM1                 100.0f
#undef M1_HFI_RUN_RPM1_S
#define M1_HFI_RUN_RPM1_S               5.0f
#undef M1_HFI_RUN_RPM2
#define M1_HFI_RUN_RPM2                 500.0f
#undef M1_HFI_RUN_RPM2_S
#define M1_HFI_RUN_RPM2_S               20.0f
#undef M1_HFI_XY_X_SIGN
#define M1_HFI_XY_X_SIGN                (-1.0f)
#undef M1_HFI_XY_Y_SIGN
#define M1_HFI_XY_Y_SIGN                (-1.0f)
#undef M1_HFI_QKICK_AFTER_LOCK_ENABLE
#define M1_HFI_QKICK_AFTER_LOCK_ENABLE  1
#undef M1_HFI_QKICK_THEN_HFI_ENABLE
#define M1_HFI_QKICK_THEN_HFI_ENABLE    1
#undef M1_HFI_QKICK_PRE_S
#define M1_HFI_QKICK_PRE_S              2.0f
#undef M1_HFI_QKICK_IQ_A
#define M1_HFI_QKICK_IQ_A               1.6f
#undef M1_HFI_QKICK_KICK_N
#define M1_HFI_QKICK_KICK_N             6000u
#undef M1_HFI_QKICK_HOLD_S
#define M1_HFI_QKICK_HOLD_S             0.4f
#undef M1_HFI_INJECT_POST_LOOP
#define M1_HFI_INJECT_POST_LOOP         1
#undef M1_HFI_DEMOD_INJ_AXIS
#define M1_HFI_DEMOD_INJ_AXIS           1
#undef M1_HFI_ID_ON_FROM_RUN_ENABLE
#define M1_HFI_ID_ON_FROM_RUN_ENABLE    1
#undef M1_HFI_SPEED_FB_ENABLE
#define M1_HFI_SPEED_FB_ENABLE          1
#undef M1_HFI_SPEED_OBS_INT
#define M1_HFI_SPEED_OBS_INT            1
#undef M1_HFI_PLL_KP
#define M1_HFI_PLL_KP                   1200.0f
#undef M1_HFI_PLL_VESC_ERR_ENABLE
#define M1_HFI_PLL_VESC_ERR_ENABLE      1
#undef M1_HFI_PLL_VESC_MAX_ERR
#define M1_HFI_PLL_VESC_MAX_ERR         (0.30f)
#undef M1_HFI_PLL_VESC_ERR_SIGN
#define M1_HFI_PLL_VESC_ERR_SIGN        (-1.0f)
#undef M1_HFI_IQ_AUTH_ENABLE
#define M1_HFI_IQ_AUTH_ENABLE           1
#undef M1_HFI_IQ_AUTH_X_GOOD
#define M1_HFI_IQ_AUTH_X_GOOD           (0.218f)
#undef M1_HFI_IQ_AUTH_X_BAD
#define M1_HFI_IQ_AUTH_X_BAD            (0.205f)
#undef M1_HFI_IQ_AUTH_EPS_FALSE_MAX
#define M1_HFI_IQ_AUTH_EPS_FALSE_MAX    (0.20f)
#undef M1_HFI_IQ_AUTH_HOLD_N
#define M1_HFI_IQ_AUTH_HOLD_N           2000u
#undef M1_HFI_IQ_AUTH_CLEAR_N
#define M1_HFI_IQ_AUTH_CLEAR_N          1000u
#undef M1_HFI_IQ_AUTH_IQ_LO
#define M1_HFI_IQ_AUTH_IQ_LO            (0.0f)
#undef M1_HFI_IQ_AUTH_IQ_HI
#define M1_HFI_IQ_AUTH_IQ_HI            (2.0f)
#undef M1_HFI_IQ_AUTH_SLEW_A_S
#define M1_HFI_IQ_AUTH_SLEW_A_S         (4.0f)
#undef M1_HFI_IQ_AUTH_FEED_ENABLE
#define M1_HFI_IQ_AUTH_FEED_ENABLE      0
#undef M1_HFI_DEMOD_HP_ENABLE
#define M1_HFI_DEMOD_HP_ENABLE          1
#undef M1_HFI_DEMOD_HP_A
#define M1_HFI_DEMOD_HP_A               (0.05f)
#undef M1_HFI_DEMOD_SKIP_OUTLIER
#define M1_HFI_DEMOD_SKIP_OUTLIER       0
#undef M1_SPEED_IQ_REF_ABS_MAX
#define M1_SPEED_IQ_REF_ABS_MAX         2.0f
#undef M1_SPEED_PI_OUT_MAX
#define M1_SPEED_PI_OUT_MAX             M1_SPEED_IQ_REF_ABS_MAX
#undef M1_SPEED_PI_OUT_MIN
#define M1_SPEED_PI_OUT_MIN             (-M1_SPEED_IQ_REF_ABS_MAX)
#undef M1_SPEED_PI_INT_MAX
#define M1_SPEED_PI_INT_MAX             M1_SPEED_IQ_REF_ABS_MAX
#undef M1_SPEED_PI_INT_MIN
#define M1_SPEED_PI_INT_MIN             (-M1_SPEED_IQ_REF_ABS_MAX)
#undef M1_SPEED_PI_KI
#define M1_SPEED_PI_KI                  (6.5e-6f)
#undef M1_SPEED_PI_BETA
#define M1_SPEED_PI_BETA                (1.0f)
#undef M1_SPEED_PI_KP
#define M1_SPEED_PI_KP                  (0.005f)
/* 签收过的 SMO 增益。127 只旁路。128 到速度窗才换发布角。旧交接状态机不开。 */
#undef M1_EMF_SMO_ENABLE
#define M1_EMF_SMO_ENABLE               1
#undef M1_EMF_PLL_ENABLE
#define M1_EMF_PLL_ENABLE               1
#undef M1_EMF_PLL_USE_SMO
#define M1_EMF_PLL_USE_SMO              1
#undef M1_HFI_SMO_SUB_VH_ENABLE
#define M1_HFI_SMO_SUB_VH_ENABLE        1
#undef M1_EMF_SMO_K
#define M1_EMF_SMO_K                    20.0f
#undef M1_EMF_SMO_SAT_A
#define M1_EMF_SMO_SAT_A                0.30f
#undef M1_EMF_SMO_LPF_ENABLE
#define M1_EMF_SMO_LPF_ENABLE           1
#undef M1_EMF_SMO_LPF_HZ
#define M1_EMF_SMO_LPF_HZ               200.0f
#undef M1_EMF_SMO_LPF_SCHED_ENABLE
#define M1_EMF_SMO_LPF_SCHED_ENABLE     0
#undef M1_EMF_SMO_LPF_LINEAR_ENABLE
#define M1_EMF_SMO_LPF_LINEAR_ENABLE    1
#undef M1_EMF_SMO_LPF_LINEAR_K
#define M1_EMF_SMO_LPF_LINEAR_K         1.2f
#undef M1_EMF_SMO_LPF_LINEAR_FC_MIN
#define M1_EMF_SMO_LPF_LINEAR_FC_MIN    100.0f
#undef M1_EMF_SMO_LPF_LINEAR_FC_MAX
#define M1_EMF_SMO_LPF_LINEAR_FC_MAX    280.0f
#undef M1_EMF_SMO_THETA_OFF_RAD
#define M1_EMF_SMO_THETA_OFF_RAD        (0.0f)
#undef M1_EMF_PLL_FN_HZ
#define M1_EMF_PLL_FN_HZ                45.0f
#undef M1_EMF_PLL_ZETA
#define M1_EMF_PLL_ZETA                 0.707106781f
#undef M1_EMF_PLL_NORM_ENABLE
#define M1_EMF_PLL_NORM_ENABLE          1
#undef M1_EMF_LPF_PHASE_FF_ENABLE
#define M1_EMF_LPF_PHASE_FF_ENABLE      1
#undef M1_EMF_PLL_THETA_OFF_RAD
#define M1_EMF_PLL_THETA_OFF_RAD        (0.1147f)

#endif /* M1_HFI_GATE == 141 */

#endif /* CONFIG_PROFILES_M1_HFI_STANDSTILL_PROFILE_H */

