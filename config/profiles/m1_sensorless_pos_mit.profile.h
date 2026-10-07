/**
 * @file m1_sensorless_pos_mit.profile.h
 * @date 2026-10-06
 * @brief 无感位置/MIT 冒烟档：HFI Park+ω̂ 外环，签收短表，精度不计。
 *
 * 打开 M1_USE_SENSORLESS_POS_MIT_PROFILE。与有感签收、纯 141 速度阶梯互斥。
 * θ_fb=HFI 多圈解包；编码器只进遥测当尺子。
 * 冒烟默认关超速守卫与 Iq 权威限幅，避免中途掐死。本文件只铺宏。
 */
#ifndef CONFIG_PROFILES_M1_SENSORLESS_POS_MIT_PROFILE_H
#define CONFIG_PROFILES_M1_SENSORLESS_POS_MIT_PROFILE_H

#ifndef M1_HFI_GATE
#define M1_HFI_GATE                     141
#endif
#if M1_HFI_GATE != 141
#error "M1_HFI_GATE must be 141 (sensorless pos smoke)"
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
#undef M1_SPEED_PI_BETA
#define M1_SPEED_PI_BETA                1.0f
#undef M1_SPEED_PI_KP
#define M1_SPEED_PI_KP                  0.005f
#undef M1_SPEED_PI_KI
#define M1_SPEED_PI_KI                  (6.5e-6f)
#undef M1_I_REF_ABS_MAX
#define M1_I_REF_ABS_MAX                5.0f
#undef M1_SPEED_IQ_REF_ABS_MAX
#define M1_SPEED_IQ_REF_ABS_MAX         5.0f
#undef M1_SPEED_PI_OUT_MAX
#define M1_SPEED_PI_OUT_MAX             5.0f
#undef M1_SPEED_PI_OUT_MIN
#define M1_SPEED_PI_OUT_MIN             (-5.0f)
#undef M1_SPEED_PI_INT_MAX
#define M1_SPEED_PI_INT_MAX             5.0f
#undef M1_SPEED_PI_INT_MIN
#define M1_SPEED_PI_INT_MIN             (-5.0f)

#undef M1_OPEN_UD_PRE_ID_CAL_ENABLE
#define M1_OPEN_UD_PRE_ID_CAL_ENABLE    0
#undef M1_OPEN_UQ_PRE_ID_CAL_ENABLE
#define M1_OPEN_UQ_PRE_ID_CAL_ENABLE    0

#define M1_DEADBAND_ENABLE              0
#define M1_DEADBAND_LUT_BAKED_ENABLE    0
#define M1_DEADBAND_NVM_ON_BOOT         0

#undef M1_STARTUP_ENABLE
#define M1_STARTUP_ENABLE               0
#undef M1_IF_ENABLE
#define M1_IF_ENABLE                    0
#undef M1_IF_TO_OBS_ENABLE
#define M1_IF_TO_OBS_ENABLE             0

#undef M1_ENC_OPTIONAL_ENABLE
#define M1_ENC_OPTIONAL_ENABLE          1

#undef M1_IQ_REF_A
#define M1_IQ_REF_A                     0.0f

/* ---- HFI 141 锁相骨架；RUN 阶梯由电流环旁路门关掉，外环接管 ---- */
#undef M1_HFI_ENABLE
#define M1_HFI_ENABLE                   1
#undef M1_HFI_MOTION_BYPASS_ENABLE
#define M1_HFI_MOTION_BYPASS_ENABLE     1
#undef M1_HFI_PARK_ENABLE
#define M1_HFI_PARK_ENABLE              1
#undef M1_HFI_SPEED_FB_ENABLE
#define M1_HFI_SPEED_FB_ENABLE          1
#undef M1_HFI_IPD_SWEEP_ENABLE
#define M1_HFI_IPD_SWEEP_ENABLE         0
#undef M1_HFI_QKICK_SWEEP_ENABLE
#define M1_HFI_QKICK_SWEEP_ENABLE       0
#undef M1_HFI_QKICK_AFTER_LOCK_ENABLE
#define M1_HFI_QKICK_AFTER_LOCK_ENABLE  1
#undef M1_HFI_QKICK_THEN_HFI_ENABLE
#define M1_HFI_QKICK_THEN_HFI_ENABLE    1
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
#undef M1_HFI_DELTA_SWEEP_ENABLE
#define M1_HFI_DELTA_SWEEP_ENABLE       0

#undef M1_HFI_BOOT_DELAY_S
#define M1_HFI_BOOT_DELAY_S             5.0f
#undef M1_HFI_RUN_LADDER_ENABLE
#define M1_HFI_RUN_LADDER_ENABLE        0
#undef M1_HFI_QKICK_PRE_S
#define M1_HFI_QKICK_PRE_S              2.0f
#undef M1_HFI_QKICK_IQ_A
#define M1_HFI_QKICK_IQ_A               1.6f
#undef M1_HFI_QKICK_KICK_N
#define M1_HFI_QKICK_KICK_N             6000u
#undef M1_HFI_QKICK_HOLD_S
#define M1_HFI_QKICK_HOLD_S             0.4f

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
#define M1_HFI_PLL_KP                   1200.0f
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
#undef M1_HFI_XY_X_SIGN
#define M1_HFI_XY_X_SIGN                (-1.0f)
#undef M1_HFI_XY_Y_SIGN
#define M1_HFI_XY_Y_SIGN                (-1.0f)
#undef M1_HFI_INJECT_POST_LOOP
#define M1_HFI_INJECT_POST_LOOP         1
#undef M1_HFI_DEMOD_INJ_AXIS
#define M1_HFI_DEMOD_INJ_AXIS           1
#undef M1_HFI_ID_ON_FROM_RUN_ENABLE
#define M1_HFI_ID_ON_FROM_RUN_ENABLE    1
#undef M1_HFI_SPEED_OBS_INT
#define M1_HFI_SPEED_OBS_INT            1
#undef M1_HFI_PLL_VESC_ERR_ENABLE
#define M1_HFI_PLL_VESC_ERR_ENABLE      1
#undef M1_HFI_PLL_VESC_MAX_ERR
#define M1_HFI_PLL_VESC_MAX_ERR         (0.30f)
#undef M1_HFI_PLL_VESC_ERR_SIGN
#define M1_HFI_PLL_VESC_ERR_SIGN        (-1.0f)
/* 冒烟关掉权威限幅：锁质量掉线时把 Iq 掐到 0 反而更容易失步飞车。 */
#undef M1_HFI_IQ_AUTH_ENABLE
#define M1_HFI_IQ_AUTH_ENABLE           0
#undef M1_HFI_IQ_AUTH_FEED_ENABLE
#define M1_HFI_IQ_AUTH_FEED_ENABLE      0
#undef M1_HFI_DEMOD_HP_ENABLE
#define M1_HFI_DEMOD_HP_ENABLE          1
#undef M1_HFI_DEMOD_HP_A
#define M1_HFI_DEMOD_HP_A               (0.05f)
#undef M1_HFI_DEMOD_SKIP_OUTLIER
#define M1_HFI_DEMOD_SKIP_OUTLIER       0

#undef M1_EMF_VEQ_ENABLE
#define M1_EMF_VEQ_ENABLE               0
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
#undef M1_OBS_SOFT_SWITCH_ENABLE
#define M1_OBS_SOFT_SWITCH_ENABLE       0
#undef M1_OBS_SPD_PLL_ENABLE
#define M1_OBS_SPD_PLL_ENABLE           0

#undef M1_PLL_ENABLE
#define M1_PLL_ENABLE                   1

#undef M1_POS_LOOP_ENABLE
#define M1_POS_LOOP_ENABLE              1
#undef M1_POS_LOOP_BOOT
#define M1_POS_LOOP_BOOT                0
#undef M1_POS_STEP_TEST_ENABLE
#define M1_POS_STEP_TEST_ENABLE         0
#undef M1_POS_MIT_COMBO_ENABLE
#define M1_POS_MIT_COMBO_ENABLE         0
#undef M1_POS_ERR_HYST_ENABLE
#define M1_POS_ERR_HYST_ENABLE          0
#undef M1_POS_DECIM
#define M1_POS_DECIM                    1u
#undef M1_POS_KP_RPM_PER_RAD
#define M1_POS_KP_RPM_PER_RAD           20.0f
#undef M1_POS_OMEGA_MAX_RPM
#define M1_POS_OMEGA_MAX_RPM            300.0f
#undef M1_SPEED_REVERSAL_TEST_ENABLE
#define M1_SPEED_REVERSAL_TEST_ENABLE   0

#undef M1_OUTER_NEST_ENABLE
#define M1_OUTER_NEST_ENABLE            1
#undef M1_OUTER_EXPT
#define M1_OUTER_EXPT                   M1_OUTER_EXPT_SIGNOFF
#undef M1_MIT_KP_SCALE
#define M1_MIT_KP_SCALE                 2.0f
#undef M1_MIT_KD_RPM
#define M1_MIT_KD_RPM                   0.0f
#undef M1_MIT_KI_DISABLE
#define M1_MIT_KI_DISABLE               1
#undef M1_OUTER_THETA_FB_SRC
#define M1_OUTER_THETA_FB_SRC           2
#undef M1_OUTER_OVERSPEED_GUARD_ENABLE
#define M1_OUTER_OVERSPEED_GUARD_ENABLE 0
#undef M1_OUTER_VEL_LIMIT_TOLERANCE
#define M1_OUTER_VEL_LIMIT_TOLERANCE    8.0f
#undef M1_OUTER_TRAJ_VMAX_RPM
#define M1_OUTER_TRAJ_VMAX_RPM          80.0f
#undef M1_OUTER_TRAJ_AMAX_RAD_S2
#define M1_OUTER_TRAJ_AMAX_RAD_S2       80.0f
#undef M1_FRIC_FF_ENABLE
#define M1_FRIC_FF_ENABLE               1
#undef M1_FRIC_FF_A
#define M1_FRIC_FF_A                    0.35f
#undef M1_FRIC_FF_SAT_RAD
#define M1_FRIC_FF_SAT_RAD              (2.0f * 0.01745329252f)
#undef M1_OUTER_SIGNOFF_PACK1
#define M1_OUTER_SIGNOFF_PACK1          1
#undef M1_OUTER_SIGNOFF_PACK2
#define M1_OUTER_SIGNOFF_PACK2          0

#undef M1_VOFA_UNIFIED_12CH
#define M1_VOFA_UNIFIED_12CH            1
#undef M1_TELEM_BRINGUP_K
#define M1_TELEM_BRINGUP_K              12u
#undef M1_TELEM_BRINGUP_DECIMATION
#define M1_TELEM_BRINGUP_DECIMATION     8u
#undef M1_VOFA_HFI_12CH
#define M1_VOFA_HFI_12CH                0
#undef M1_VOFA_PLL_CH8_11
#define M1_VOFA_PLL_CH8_11              0
#undef M1_VOFA_SPEED_CH8_11
#define M1_VOFA_SPEED_CH8_11            0
#undef M1_VOFA_MIT_CH8_11
#define M1_VOFA_MIT_CH8_11              0
#undef M1_VOFA_SIGNOFF_CH
#define M1_VOFA_SIGNOFF_CH              1
#undef M1_CLOSURE_BRINGUP
#define M1_CLOSURE_BRINGUP              0

#endif /* CONFIG_PROFILES_M1_SENSORLESS_POS_MIT_PROFILE_H */
