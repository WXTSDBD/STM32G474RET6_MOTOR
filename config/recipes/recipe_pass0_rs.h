/**
 * @file recipe_pass0_rs.h
 * @date 2026-10-31
 * @brief PASS0 Rs/Ld/Lq 标定配方（route 9）。纯宏，无嵌套 profile。
 * @note defaults_id_cal 已先纳入；本头 #undef 再覆盖。
 */
#ifndef CONFIG_RECIPES_RECIPE_PASS0_RS_H
#define CONFIG_RECIPES_RECIPE_PASS0_RS_H

/** 只跑参数辨识：跳过 Pass0/LUT，ALIGN → Rs ramp → l_hfi×10（VESC 旋转HF+DFT）。
 * defaults_id_cal 在 motor_params 算出 PI 限幅后纳入（依赖 M1_PI_V_LIMIT_V）。 */
#undef M1_ID_LOCK_CAL_SWEEP
#define M1_ID_LOCK_CAL_SWEEP            1
#undef M1_IDENT_ENABLE
#define M1_IDENT_ENABLE                 0
#undef M1_SPEED_IDENT_ENABLE
#define M1_SPEED_IDENT_ENABLE           0
#undef M1_SPEED_LOOP_ENABLE
#define M1_SPEED_LOOP_ENABLE            0
#undef M1_PLL_ENABLE
#define M1_PLL_ENABLE                   0
#undef M1_VOFA_PLL_CH8_11
#define M1_VOFA_PLL_CH8_11              0
#undef M1_VOFA_IDENT_DUTY_12CH
#define M1_VOFA_IDENT_DUTY_12CH         1
#undef M1_ID_CAL_RS_LD_LQ_ONLY_ENABLE
#define M1_ID_CAL_RS_LD_LQ_ONLY_ENABLE  1
#undef M1_ID_CAL_MULTI_ANGLE_ENABLE
#define M1_ID_CAL_MULTI_ANGLE_ENABLE    0
#undef M1_ID_CAL_PASS0_ONLY_ENABLE
#define M1_ID_CAL_PASS0_ONLY_ENABLE     0
#undef M1_ID_CAL_LUT_VERIFY_SWEEP
#define M1_ID_CAL_LUT_VERIFY_SWEEP      0
#undef M1_ID_CAL_IQ_PROBE_ENABLE
#define M1_ID_CAL_IQ_PROBE_ENABLE       0
/** Rs 最小二乘；与 FIXED_NOMINAL 互斥。 */
#undef M1_RS_IDENT_USE_FIXED_NOMINAL
#define M1_RS_IDENT_USE_FIXED_NOMINAL   0
#undef M1_RS_IDENT_ENABLE
#define M1_RS_IDENT_ENABLE              1
#undef M1_L_HFI_IDENT_ENABLE
#define M1_L_HFI_IDENT_ENABLE           1
#undef M1_ID_CAL_DUAL_ANGLE_ENABLE
#define M1_ID_CAL_DUAL_ANGLE_ENABLE     0
#undef M1_ID_CAL_FIX_THETA_ENABLE
#define M1_ID_CAL_FIX_THETA_ENABLE      1
/** 开环 Ud 拉到 FIX_THETA 角，再 Rs 灌 Id。 */
#undef M1_ID_CAL_ALIGN_ENABLE
#define M1_ID_CAL_ALIGN_ENABLE          1
/** 3V 曾拉到 ~11A；1.5V 够贴角，电流约减半。 */
#undef M1_ID_CAL_ALIGN_UD_V
#define M1_ID_CAL_ALIGN_UD_V            1.5f
#undef M1_ID_CAL_COMMIT_LUT
#define M1_ID_CAL_COMMIT_LUT            0
#undef M1_RS_L_IDENT_DUAL_LUT_ROUND_ENABLE
#define M1_RS_L_IDENT_DUAL_LUT_ROUND_ENABLE  0
#undef M1_DEADBAND_ENABLE
#define M1_DEADBAND_ENABLE              0
/** HF-L 连跑 10 轮，轮间歇 2 s；每轮 mark55 连发本轮结果。 */
#undef M1_L_HFI_REPEAT_N
#define M1_L_HFI_REPEAT_N               10u
#undef M1_L_HFI_INTER_ROUND_S
#define M1_L_HFI_INTER_ROUND_S          2.0f
/*
 * 遥测：6ch @ 20kHz（DECIM=1）。12ch@10kHz 对 20kHz 双极半拍别名，离线凑不齐 Δi；
 * 6×20k≈560KB/s，与原 12×10k 同量级，6Mbps UART 吃得下。VOFA 通道数改 6。
 */
#undef M1_TELEM_BRINGUP_K
#define M1_TELEM_BRINGUP_K              6u
#undef M1_TELEM_BRINGUP_DECIMATION
#define M1_TELEM_BRINGUP_DECIMATION     1u

#undef M1_HFI_ENABLE
#define M1_HFI_ENABLE                   1
#undef M1_HFI_MOTION_BYPASS_ENABLE
#define M1_HFI_MOTION_BYPASS_ENABLE     0
#undef M1_HFI_VH_V
#define M1_HFI_VH_V                     0.40f
#undef M1_HFI_FH_HZ
#define M1_HFI_FH_HZ                    10000.0f
#undef M1_HFI_PLL_ENABLE
#define M1_HFI_PLL_ENABLE               1
#undef M1_HFI_SPEED_FB_ENABLE
#define M1_HFI_SPEED_FB_ENABLE          0
#undef M1_HFI_ATAN2_ENABLE
#define M1_HFI_ATAN2_ENABLE             0
#undef M1_HFI_A_CMD
#define M1_HFI_A_CMD                    (0.10f)
#undef M1_HFI_XY_X_SIGN
#define M1_HFI_XY_X_SIGN                (-1.0f)
#undef M1_HFI_XY_Y_SIGN
#define M1_HFI_XY_Y_SIGN                (-1.0f)
#undef M1_HFI_QKICK_AFTER_LOCK_ENABLE
#define M1_HFI_QKICK_AFTER_LOCK_ENABLE  0
#undef M1_HFI_QKICK_PRE_S
#define M1_HFI_QKICK_PRE_S              2.0f
#undef M1_HFI_QKICK_IQ_A
#define M1_HFI_QKICK_IQ_A               1.6f
#undef M1_HFI_QKICK_KICK_N
#define M1_HFI_QKICK_KICK_N             6000u
#undef M1_HFI_QKICK_HOLD_S
#define M1_HFI_QKICK_HOLD_S             0.6f
#undef M1_HFI_QKICK_HAT0_N
#define M1_HFI_QKICK_HAT0_N             0u
#undef M1_HFI_QKICK_BRAKE_N
#define M1_HFI_QKICK_BRAKE_N            0u
#undef M1_HFI_QKICK_POL_W_MIN_RAD_S
#define M1_HFI_QKICK_POL_W_MIN_RAD_S    5.0f
#undef M1_HFI_PLL_VESC_ERR_ENABLE
#define M1_HFI_PLL_VESC_ERR_ENABLE      1
#undef M1_HFI_PLL_VESC_MAX_ERR
#define M1_HFI_PLL_VESC_MAX_ERR         (0.30f)
#undef M1_HFI_PLL_VESC_ERR_SIGN
#define M1_HFI_PLL_VESC_ERR_SIGN        (-1.0f)
#undef M1_HFI_IQ_AUTH_ENABLE
#define M1_HFI_IQ_AUTH_ENABLE           0
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
#undef M1_HFI_DEMOD_HP_A
#define M1_HFI_DEMOD_HP_A               (0.05f)
#undef M1_HFI_BOOT_DELAY_S
#define M1_HFI_BOOT_DELAY_S             5.0f
#undef M1_HFI_DEMOD_INJ_AXIS
#define M1_HFI_DEMOD_INJ_AXIS           1
#undef M1_HFI_PUB_UP_RPM
#define M1_HFI_PUB_UP_RPM               1300.0f
#undef M1_HFI_PUB_DOWN_RPM
#define M1_HFI_PUB_DOWN_RPM             1000.0f
#undef M1_HFI_PUB_INJ_OFF_RPM
#define M1_HFI_PUB_INJ_OFF_RPM          1500.0f
#undef M1_HFI_PUB_INJ_ON_RPM
#define M1_HFI_PUB_INJ_ON_RPM           1400.0f

#undef M1_EMF_SMO_R_OHM
#define M1_EMF_SMO_R_OHM                0.045f
#undef M1_EMF_SMO_L_H
#define M1_EMF_SMO_L_H                  70e-6f
#undef M1_EMF_SMO_K
#define M1_EMF_SMO_K                    20.0f
#undef M1_EMF_SMO_SAT_A
#define M1_EMF_SMO_SAT_A                0.30f
#undef M1_EMF_SMO_THETA_OFF_RAD
#define M1_EMF_SMO_THETA_OFF_RAD        (0.0f)
#undef M1_EMF_SMO_LPF_HZ
#define M1_EMF_SMO_LPF_HZ               200.0f
#undef M1_EMF_SMO_LPF_LINEAR_K
#define M1_EMF_SMO_LPF_LINEAR_K         1.2f
#undef M1_EMF_SMO_LPF_LINEAR_FC_MIN
#define M1_EMF_SMO_LPF_LINEAR_FC_MIN    100.0f
#undef M1_EMF_SMO_LPF_LINEAR_FC_MAX
#define M1_EMF_SMO_LPF_LINEAR_FC_MAX    420.0f
#undef M1_EMF_PLL_FN_HZ
#define M1_EMF_PLL_FN_HZ                45.0f
#undef M1_EMF_PLL_ZETA
#define M1_EMF_PLL_ZETA                 0.707106781f
#undef M1_EMF_PLL_THETA_OFF_RAD
#define M1_EMF_PLL_THETA_OFF_RAD        (0.1147f)
#undef M1_EMF_PLL_NORM_EPS
#define M1_EMF_PLL_NORM_EPS             1.0e-3f
#undef M1_EMF_PLL_OMEGA_LIMIT_RAD_S
#define M1_EMF_PLL_OMEGA_LIMIT_RAD_S    3000.0f
#undef M1_EMF_PLL_INTEGRATOR_LIMIT_RAD_S
#define M1_EMF_PLL_INTEGRATOR_LIMIT_RAD_S 3000.0f

#undef M1_TELEM_BRINGUP_K
#define M1_TELEM_BRINGUP_K              12u
#undef M1_TELEM_BRINGUP_DECIMATION
#define M1_TELEM_BRINGUP_DECIMATION     2u
#undef M1_OUTER_IN_TIM6
#define M1_OUTER_IN_TIM6                1

#undef M1_HFI_ENABLE
#define M1_HFI_ENABLE                   1
#undef M1_HFI_MOTION_BYPASS_ENABLE
#define M1_HFI_MOTION_BYPASS_ENABLE     1
#undef M1_EMF_SMO_ENABLE
#define M1_EMF_SMO_ENABLE               1
#undef M1_EMF_PLL_ENABLE
#define M1_EMF_PLL_ENABLE               1
#undef M1_EMF_PLL_USE_SMO
#define M1_EMF_PLL_USE_SMO              1
#undef M1_VOFA_OPT12
#define M1_VOFA_OPT12                   0
#undef M1_VOFA_IDENT_DUTY_12CH
#define M1_VOFA_IDENT_DUTY_12CH         1

/* 配方互斥（defaults 先纳入时未开 Rs；覆盖后在此断言）。 */
#if M1_RS_IDENT_USE_FIXED_NOMINAL && M1_RS_IDENT_ENABLE
#error "M1_RS_IDENT_USE_FIXED_NOMINAL and M1_RS_IDENT_ENABLE are mutually exclusive"
#endif
#if M1_RS_IDENT_ENABLE && !M1_ID_CAL_COMMIT_LUT && !M1_ID_CAL_RS_LD_LQ_ONLY_ENABLE
#error "M1_RS_IDENT_ENABLE requires M1_ID_CAL_COMMIT_LUT=1 (or RS_LD_LQ_ONLY)"
#endif
#if M1_RS_IDENT_ENABLE && M1_IDENT_ENABLE
#error "M1_RS_IDENT_ENABLE requires M1_IDENT_ENABLE=0"
#endif
#if M1_L_HFI_IDENT_ENABLE && !M1_RS_IDENT_ENABLE && !M1_RS_IDENT_USE_FIXED_NOMINAL
#error "M1_L_HFI_IDENT_ENABLE requires M1_RS_IDENT_ENABLE=1 or M1_RS_IDENT_USE_FIXED_NOMINAL=1"
#endif
#if M1_L_HFI_IDENT_ENABLE && !M1_ID_CAL_COMMIT_LUT && !M1_ID_CAL_RS_LD_LQ_ONLY_ENABLE
#error "M1_L_HFI_IDENT_ENABLE requires M1_ID_CAL_COMMIT_LUT=1 (or RS_LD_LQ_ONLY)"
#endif
#if M1_ID_CAL_RS_LD_LQ_ONLY_ENABLE && !M1_ID_LOCK_CAL_SWEEP
#error "M1_ID_CAL_RS_LD_LQ_ONLY_ENABLE requires M1_ID_LOCK_CAL_SWEEP=1"
#endif
#if M1_ID_CAL_RS_LD_LQ_ONLY_ENABLE && \
    (!M1_L_HFI_IDENT_ENABLE || (!M1_RS_IDENT_ENABLE && !M1_RS_IDENT_USE_FIXED_NOMINAL))
#error "M1_ID_CAL_RS_LD_LQ_ONLY_ENABLE requires M1_L_HFI_IDENT_ENABLE and (M1_RS_IDENT_ENABLE or M1_RS_IDENT_USE_FIXED_NOMINAL)"
#endif
#if M1_ID_CAL_ALIGN_ENABLE && !M1_ID_CAL_FIX_THETA_ENABLE
#error "M1_ID_CAL_ALIGN_ENABLE requires M1_ID_CAL_FIX_THETA_ENABLE=1"
#endif

#endif /* CONFIG_RECIPES_RECIPE_PASS0_RS_H */
