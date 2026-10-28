/**
 * @file m1_hfi_compile_nums.h
 * @date 2026-10-26
 * @brief 仅提供 HFI/观测编译所需数值宏（不清 ID_LOCK/Rs/L）。
 *
 * 给 PASS0_RS 等标定 bringup 用：obs_num.c / hfi_sqwave.c 仍引用这些宏。
 * 全部 #ifndef，不覆盖标定开关。遥测抽稀默认 10 kHz。
 */
#ifndef CONFIG_PROFILES_M1_HFI_COMPILE_NUMS_H
#define CONFIG_PROFILES_M1_HFI_COMPILE_NUMS_H

#ifndef M1_HFI_ENABLE
#define M1_HFI_ENABLE                   1
#endif
#ifndef M1_HFI_MOTION_BYPASS_ENABLE
#define M1_HFI_MOTION_BYPASS_ENABLE     0
#endif
#ifndef M1_HFI_VH_V
#define M1_HFI_VH_V                     0.40f
#endif
#ifndef M1_HFI_FH_HZ
#define M1_HFI_FH_HZ                    10000.0f
#endif
#ifndef M1_HFI_PLL_ENABLE
#define M1_HFI_PLL_ENABLE               1
#endif
#ifndef M1_HFI_SPEED_FB_ENABLE
#define M1_HFI_SPEED_FB_ENABLE          0
#endif
#ifndef M1_HFI_ATAN2_ENABLE
#define M1_HFI_ATAN2_ENABLE             0
#endif
#ifndef M1_HFI_A_CMD
#define M1_HFI_A_CMD                    (0.10f)
#endif
#ifndef M1_HFI_XY_X_SIGN
#define M1_HFI_XY_X_SIGN                (-1.0f)
#endif
#ifndef M1_HFI_XY_Y_SIGN
#define M1_HFI_XY_Y_SIGN                (-1.0f)
#endif
#ifndef M1_HFI_QKICK_AFTER_LOCK_ENABLE
#define M1_HFI_QKICK_AFTER_LOCK_ENABLE  0
#endif
#ifndef M1_HFI_QKICK_PRE_S
#define M1_HFI_QKICK_PRE_S              2.0f
#endif
#ifndef M1_HFI_QKICK_IQ_A
#define M1_HFI_QKICK_IQ_A               1.6f
#endif
#ifndef M1_HFI_QKICK_KICK_N
#define M1_HFI_QKICK_KICK_N             6000u
#endif
#ifndef M1_HFI_QKICK_HOLD_S
#define M1_HFI_QKICK_HOLD_S             0.6f
#endif
#ifndef M1_HFI_QKICK_HAT0_N
#define M1_HFI_QKICK_HAT0_N             0u
#endif
#ifndef M1_HFI_QKICK_BRAKE_N
#define M1_HFI_QKICK_BRAKE_N            0u
#endif
#ifndef M1_HFI_QKICK_POL_W_MIN_RAD_S
#define M1_HFI_QKICK_POL_W_MIN_RAD_S    5.0f
#endif
#ifndef M1_HFI_PLL_VESC_ERR_ENABLE
#define M1_HFI_PLL_VESC_ERR_ENABLE      1
#endif
#ifndef M1_HFI_PLL_VESC_MAX_ERR
#define M1_HFI_PLL_VESC_MAX_ERR         (0.30f)
#endif
#ifndef M1_HFI_PLL_VESC_ERR_SIGN
#define M1_HFI_PLL_VESC_ERR_SIGN        (-1.0f)
#endif
#ifndef M1_HFI_IQ_AUTH_ENABLE
#define M1_HFI_IQ_AUTH_ENABLE           0
#endif
#ifndef M1_HFI_IQ_AUTH_X_GOOD
#define M1_HFI_IQ_AUTH_X_GOOD           (0.218f)
#endif
#ifndef M1_HFI_IQ_AUTH_X_BAD
#define M1_HFI_IQ_AUTH_X_BAD            (0.205f)
#endif
#ifndef M1_HFI_IQ_AUTH_EPS_FALSE_MAX
#define M1_HFI_IQ_AUTH_EPS_FALSE_MAX    (0.20f)
#endif
#ifndef M1_HFI_IQ_AUTH_HOLD_N
#define M1_HFI_IQ_AUTH_HOLD_N           2000u
#endif
#ifndef M1_HFI_IQ_AUTH_CLEAR_N
#define M1_HFI_IQ_AUTH_CLEAR_N          1000u
#endif
#ifndef M1_HFI_IQ_AUTH_IQ_LO
#define M1_HFI_IQ_AUTH_IQ_LO            (0.0f)
#endif
#ifndef M1_HFI_IQ_AUTH_IQ_HI
#define M1_HFI_IQ_AUTH_IQ_HI            (2.0f)
#endif
#ifndef M1_HFI_IQ_AUTH_SLEW_A_S
#define M1_HFI_IQ_AUTH_SLEW_A_S         (4.0f)
#endif
#ifndef M1_HFI_DEMOD_HP_A
#define M1_HFI_DEMOD_HP_A               (0.05f)
#endif
#ifndef M1_HFI_BOOT_DELAY_S
#define M1_HFI_BOOT_DELAY_S             5.0f
#endif
#ifndef M1_HFI_DEMOD_INJ_AXIS
#define M1_HFI_DEMOD_INJ_AXIS           1
#endif
#ifndef M1_HFI_PUB_UP_RPM
#define M1_HFI_PUB_UP_RPM               1300.0f
#endif
#ifndef M1_HFI_PUB_DOWN_RPM
#define M1_HFI_PUB_DOWN_RPM             1000.0f
#endif
#ifndef M1_HFI_PUB_INJ_OFF_RPM
#define M1_HFI_PUB_INJ_OFF_RPM          1500.0f
#endif
#ifndef M1_HFI_PUB_INJ_ON_RPM
#define M1_HFI_PUB_INJ_ON_RPM           1400.0f
#endif

#ifndef M1_EMF_SMO_R_OHM
#define M1_EMF_SMO_R_OHM                0.045f
#endif
#ifndef M1_EMF_SMO_L_H
#define M1_EMF_SMO_L_H                  70e-6f
#endif
#ifndef M1_EMF_SMO_K
#define M1_EMF_SMO_K                    20.0f
#endif
#ifndef M1_EMF_SMO_SAT_A
#define M1_EMF_SMO_SAT_A                0.30f
#endif
#ifndef M1_EMF_SMO_THETA_OFF_RAD
#define M1_EMF_SMO_THETA_OFF_RAD        (0.0f)
#endif
#ifndef M1_EMF_SMO_LPF_HZ
#define M1_EMF_SMO_LPF_HZ               200.0f
#endif
#ifndef M1_EMF_SMO_LPF_LINEAR_K
#define M1_EMF_SMO_LPF_LINEAR_K         1.2f
#endif
#ifndef M1_EMF_SMO_LPF_LINEAR_FC_MIN
#define M1_EMF_SMO_LPF_LINEAR_FC_MIN    100.0f
#endif
#ifndef M1_EMF_SMO_LPF_LINEAR_FC_MAX
#define M1_EMF_SMO_LPF_LINEAR_FC_MAX    420.0f
#endif
#ifndef M1_EMF_PLL_FN_HZ
#define M1_EMF_PLL_FN_HZ                45.0f
#endif
#ifndef M1_EMF_PLL_ZETA
#define M1_EMF_PLL_ZETA                 0.707106781f
#endif
#ifndef M1_EMF_PLL_THETA_OFF_RAD
#define M1_EMF_PLL_THETA_OFF_RAD        (0.1147f)
#endif
#ifndef M1_EMF_PLL_NORM_EPS
#define M1_EMF_PLL_NORM_EPS             1.0e-3f
#endif
#ifndef M1_EMF_PLL_OMEGA_LIMIT_RAD_S
#define M1_EMF_PLL_OMEGA_LIMIT_RAD_S    3000.0f
#endif
#ifndef M1_EMF_PLL_INTEGRATOR_LIMIT_RAD_S
#define M1_EMF_PLL_INTEGRATOR_LIMIT_RAD_S 3000.0f
#endif

#ifndef M1_TELEM_BRINGUP_K
#define M1_TELEM_BRINGUP_K              12u
#endif
#ifndef M1_TELEM_BRINGUP_DECIMATION
#define M1_TELEM_BRINGUP_DECIMATION     2u
#endif
#ifndef M1_OUTER_IN_TIM6
#define M1_OUTER_IN_TIM6                1
#endif

#endif /* CONFIG_PROFILES_M1_HFI_COMPILE_NUMS_H */
