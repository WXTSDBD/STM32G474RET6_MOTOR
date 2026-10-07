/**
 * @file m1_sensed_pos_mit.profile.h
 * @date 2026-10-06
 * @brief 位置环 / MIT 签收档：短表 + MIT 连续饱和前馈。PACK2 爬表默认关。
 *
 * 打开 M1_USE_SENSED_POS_MIT_PROFILE。与 HFI 141 互斥。
 * 一次只选一个 M1_OUTER_EXPT。本文件只铺宏。
 */
#ifndef CONFIG_PROFILES_M1_SENSED_POS_MIT_PROFILE_H
#define CONFIG_PROFILES_M1_SENSED_POS_MIT_PROFILE_H

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
#define M1_SPEED_LOOP_BOOT              1
#undef M1_SPEED_REF_RPM_DEFAULT
#define M1_SPEED_REF_RPM_DEFAULT        300.0f
#undef M1_SPEED_OMEGA_RAMP_ENABLE
#define M1_SPEED_OMEGA_RAMP_ENABLE      0
#undef M1_SPEED_PI_BETA
#define M1_SPEED_PI_BETA                1.0f
#undef M1_SPEED_PI_KP
#define M1_SPEED_PI_KP                  0.015f
#undef M1_SPEED_PI_KI
#define M1_SPEED_PI_KI                  0.002f
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
#define M1_ENC_OPTIONAL_ENABLE          0

#undef M1_HFI_ENABLE
#define M1_HFI_ENABLE                   0
#undef M1_HFI_PARK_ENABLE
#define M1_HFI_PARK_ENABLE              0
#undef M1_HFI_MOTION_BYPASS_ENABLE
#define M1_HFI_MOTION_BYPASS_ENABLE     0
#undef M1_HFI_SPEED_FB_ENABLE
#define M1_HFI_SPEED_FB_ENABLE          0

#undef M1_EMF_VEQ_ENABLE
#define M1_EMF_VEQ_ENABLE               0
#undef M1_EMF_SMO_ENABLE
#define M1_EMF_SMO_ENABLE               0
#undef M1_EMF_PLL_ENABLE
#define M1_EMF_PLL_ENABLE               0
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
#define M1_POS_KP_RPM_PER_RAD           60.0f
#undef M1_POS_OMEGA_MAX_RPM
#define M1_POS_OMEGA_MAX_RPM            250.0f
#undef M1_SPEED_REVERSAL_TEST_ENABLE
#define M1_SPEED_REVERSAL_TEST_ENABLE   0

#undef M1_OUTER_NEST_ENABLE
#define M1_OUTER_NEST_ENABLE            1
#undef M1_OUTER_EXPT
#define M1_OUTER_EXPT                   M1_OUTER_EXPT_SIGNOFF
#undef M1_MIT_KP_SCALE
#define M1_MIT_KP_SCALE                 4.0f
#undef M1_MIT_KD_RPM
#define M1_MIT_KD_RPM                   0.0f
#undef M1_MIT_KI_DISABLE
#define M1_MIT_KI_DISABLE               1
#undef M1_OUTER_THETA_FB_SRC
#define M1_OUTER_THETA_FB_SRC           0
#undef M1_FRIC_FF_ENABLE
#define M1_FRIC_FF_ENABLE               1
#undef M1_FRIC_FF_A
#define M1_FRIC_FF_A                    0.35f
#undef M1_FRIC_FF_SAT_RAD
#define M1_FRIC_FF_SAT_RAD              (2.0f * 0.01745329252f)
#undef M1_OUTER_SIGNOFF_PACK1
#define M1_OUTER_SIGNOFF_PACK1          0
#undef M1_OUTER_SIGNOFF_PACK2
#define M1_OUTER_SIGNOFF_PACK2          0
#undef M1_EXP_FRAMEWORK_ENABLE
#define M1_EXP_FRAMEWORK_ENABLE         1

#undef M1_VOFA_UNIFIED_12CH
#define M1_VOFA_UNIFIED_12CH            1
#undef M1_TELEM_BRINGUP_K
#define M1_TELEM_BRINGUP_K              12u
#undef M1_TELEM_BRINGUP_DECIMATION
#define M1_TELEM_BRINGUP_DECIMATION     8u
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

#endif /* CONFIG_PROFILES_M1_SENSED_POS_MIT_PROFILE_H */
