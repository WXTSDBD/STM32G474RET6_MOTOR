/**
 * @file m1_obs_veq_1000rpm.profile.h
 * @date 2026-10-06
 * @brief 电压方程观测器旁路对照 profile。

 *
 * 默认不进 Park。只铺宏。
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
 */

#ifndef CONFIG_PROFILES_M1_OBS_VEQ_1000RPM_PROFILE_H
#define CONFIG_PROFILES_M1_OBS_VEQ_1000RPM_PROFILE_H

#define M1_ID_LOCK_CAL_SWEEP            0
#define M1_IDENT_ENABLE                 0
#define M1_ID_CAL_ALIGN_ENABLE          0
#define M1_ID_CAL_COMMIT_LUT            0
#define M1_LD_LQ_IDENT_ENABLE           0
#define M1_RS_IDENT_ENABLE              0
#define M1_VOFA_IDENT_DUMP_ENABLE       0
#define M1_SPEED_IDENT_ENABLE           1

#undef M1_OPEN_UD_PRE_ID_CAL_ENABLE
#define M1_OPEN_UD_PRE_ID_CAL_ENABLE    0
#undef M1_OPEN_UQ_PRE_ID_CAL_ENABLE
#define M1_OPEN_UQ_PRE_ID_CAL_ENABLE    0

#define M1_DEADBAND_ENABLE              0
#define M1_DEADBAND_LUT_BAKED_ENABLE    0
#define M1_DEADBAND_NVM_ON_BOOT         0
#define M1_SPEED_PROFILE_ENABLE         0

#define M1_SPEED_IDENT_STEP_ENABLE      1
#define M1_SPEED_IDENT_BODE_ENABLE      0

#undef M1_SPEED_IDENT_STEP_LADDER_ENABLE
#define M1_SPEED_IDENT_STEP_LADDER_ENABLE  1

#define M1_SPEED_IDENT_HOLD_S           8.0f
#ifndef M1_SPEED_IDENT_RPM_START
#define M1_SPEED_IDENT_RPM_START        100.0f
#endif
#define M1_SPEED_IDENT_STEP_ROUNDS      1u
#define M1_SPEED_IDENT_STEP_RPM0        400.0f
#define M1_SPEED_IDENT_STEP_RPM1        500.0f
#define M1_SPEED_IDENT_STEP_RPM2        600.0f
#define M1_SPEED_IDENT_STEP_RPM3        700.0f
#define M1_SPEED_IDENT_STEP_RPM4        800.0f
#define M1_SPEED_IDENT_STEP_RPM5        900.0f
#define M1_SPEED_IDENT_STEP_RPM6        1000.0f
#define M1_SPEED_IDENT_STEP_DWELL_S     10.0f
#define M1_SPEED_IDENT_STEP_ZERO_DWELL_S 10.0f

#ifndef M1_SPEED_IDENT_PLL_SETTLE_S
#define M1_SPEED_IDENT_PLL_SETTLE_S     1.5f
#endif

#undef M1_VOFA_PLL_CH8_11
#define M1_VOFA_PLL_CH8_11              0
#undef M1_VOFA_SPEED_CH8_11
#define M1_VOFA_SPEED_CH8_11            0

#undef M1_VOFA_UNIFIED_12CH
#define M1_VOFA_UNIFIED_12CH            1
#undef M1_TELEM_BRINGUP_K
#define M1_TELEM_BRINGUP_K              12u
#undef M1_TELEM_BRINGUP_DECIMATION
#define M1_TELEM_BRINGUP_DECIMATION     2u

#undef M1_VOFA_FLUX_ID_6CH
#define M1_VOFA_FLUX_ID_6CH             0
#undef M1_VOFA_CH11_ENC_RAW
#define M1_VOFA_CH11_ENC_RAW            0

#undef M1_EMF_VEQ_ENABLE
#define M1_EMF_VEQ_ENABLE               1
#undef M1_EMF_SMO_ENABLE
#define M1_EMF_SMO_ENABLE               0

#undef M1_VOFA_OBS_SMO_12CH
#define M1_VOFA_OBS_SMO_12CH            0
#undef M1_VOFA_OBS_VEQ_12CH
#define M1_VOFA_OBS_VEQ_12CH            1

#undef M1_EMF_VEQ_LPF_ENABLE
#define M1_EMF_VEQ_LPF_ENABLE           1
#undef M1_EMF_VEQ_LPF_HZ
#define M1_EMF_VEQ_LPF_HZ               200.0f
#undef M1_EMF_VEQ_THETA_OFF_RAD
#define M1_EMF_VEQ_THETA_OFF_RAD        (-0.4054f)

#undef M1_CLOSURE_BRINGUP
#define M1_CLOSURE_BRINGUP              0

#endif /* CONFIG_PROFILES_M1_OBS_VEQ_1000RPM_PROFILE_H */
