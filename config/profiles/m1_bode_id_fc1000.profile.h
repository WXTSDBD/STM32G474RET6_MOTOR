/**
 * @file m1_bode_id_fc1000.profile.h
 * @date 2026-10-06
 * @brief 堵转 Bode 扫频 profile。

 *
 * 只铺宏。切实验改 bringup_active.h。
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
 */

#ifndef CONFIG_PROFILES_M1_BODE_ID_FC1000_PROFILE_H
#define CONFIG_PROFILES_M1_BODE_ID_FC1000_PROFILE_H

/** PI 设计带宽（签收基线） */
#undef M1_PI_FC_HZ
#define M1_PI_FC_HZ                     1000.0f

#define M1_ID_LOCK_CAL_SWEEP            0
#define M1_IDENT_ID_CAL_BEFORE_STEP     0
#define M1_IDENT_ENABLE                 1
#define M1_IDENT_IQ_STEP_ENABLE         0
#define M1_IDENT_IQ_BODE_ENABLE         1
#undef M1_IDENT_STEP_ROUNDS
#define M1_IDENT_STEP_ROUNDS            0u
#undef M1_IDENT_STEP_OFF_ROUNDS
#define M1_IDENT_STEP_OFF_ROUNDS        0u
#undef M1_IDENT_STEP_FIXED_ROUNDS
#define M1_IDENT_STEP_FIXED_ROUNDS      0u
#define M1_RS_IDENT_ENABLE              0
#define M1_LD_LQ_IDENT_ENABLE           0
#define M1_SPEED_IDENT_ENABLE           0

#undef M1_IDENT_BODE_LUT_ENABLE
#define M1_IDENT_BODE_LUT_ENABLE        0
#undef M1_IDENT_BODE_BANDS
#define M1_IDENT_BODE_BANDS             2u
#undef M1_IDENT_BODE_ROUNDS
#define M1_IDENT_BODE_ROUNDS            2u
#undef M1_IDENT_BODE_OFF_ROUNDS
#define M1_IDENT_BODE_OFF_ROUNDS        2u
#undef M1_IDENT_BODE_FIXED_ROUNDS
#define M1_IDENT_BODE_FIXED_ROUNDS      0u

#undef M1_IDENT_BODE_I_BIAS_A
#define M1_IDENT_BODE_I_BIAS_A          0.25f
#undef M1_IDENT_BODE_I_AMP_A
#define M1_IDENT_BODE_I_AMP_A           0.05f
#undef M1_IDENT_BODE_I_BIAS_HI_A
#define M1_IDENT_BODE_I_BIAS_HI_A       1.25f
#undef M1_IDENT_BODE_I_AMP_HI_A
#define M1_IDENT_BODE_I_AMP_HI_A        0.10f

#undef M1_IDENT_BODE_CYCLES_PER_FREQ
#define M1_IDENT_BODE_CYCLES_PER_FREQ   20.0f
#undef M1_IDENT_BODE_CYCLES_HI
#define M1_IDENT_BODE_CYCLES_HI         50.0f
#undef M1_IDENT_BODE_F0_HZ
#define M1_IDENT_BODE_F0_HZ             10.0f
#undef M1_IDENT_BODE_F1_HZ
#define M1_IDENT_BODE_F1_HZ             5000.0f
#undef M1_IDENT_BODE_F_RATIO
#define M1_IDENT_BODE_F_RATIO           1.15f
#undef M1_IDENT_BODE_F_SPLIT_HZ
#define M1_IDENT_BODE_F_SPLIT_HZ        500.0f
#undef M1_IDENT_BODE_USE_T_OBS_HI
#define M1_IDENT_BODE_USE_T_OBS_HI      1
#undef M1_IDENT_BODE_T_OBS_HI_S
#define M1_IDENT_BODE_T_OBS_HI_S        0.1f
#undef M1_IDENT_BODE_F_RATIO_HI
#define M1_IDENT_BODE_F_RATIO_HI        1.06f

#undef M1_IDENT_BODE_AXIS_ID
#define M1_IDENT_BODE_AXIS_ID           1
#undef M1_IDENT_FIX_THETA_ENABLE
#define M1_IDENT_FIX_THETA_ENABLE       1
#undef M1_IDENT_THETA_EL_RAD
#define M1_IDENT_THETA_EL_RAD           M1_ID_CAL_THETA_EL_RAD
#undef M1_IDENT_HOLD_S
#define M1_IDENT_HOLD_S                 2.0f
#undef M1_IDENT_OVERRIDE_LIMITS
#define M1_IDENT_OVERRIDE_LIMITS        1

#undef M1_OPEN_UD_PRE_ID_CAL_ENABLE
#define M1_OPEN_UD_PRE_ID_CAL_ENABLE    0
#undef M1_OPEN_UQ_PRE_ID_CAL_ENABLE
#define M1_OPEN_UQ_PRE_ID_CAL_ENABLE    0

#define M1_DEADBAND_LUT_BAKED_ENABLE    0
#define M1_DEADBAND_ENABLE              1

#undef M1_VOFA_UNIFIED_12CH
#define M1_VOFA_UNIFIED_12CH            0
#undef M1_TELEM_BRINGUP_K
#define M1_TELEM_BRINGUP_K              6u
#undef M1_TELEM_BRINGUP_DECIMATION
#define M1_TELEM_BRINGUP_DECIMATION     1u
#undef M1_VOFA_LUT_DUMP_ENABLE
#define M1_VOFA_LUT_DUMP_ENABLE         0
#undef M1_CLOSURE_BRINGUP
#define M1_CLOSURE_BRINGUP              0
#define M1_IDENT_POST_BODE_OPEN_UQ_ENABLE  0

#endif /* CONFIG_PROFILES_M1_BODE_ID_FC1000_PROFILE_H */
