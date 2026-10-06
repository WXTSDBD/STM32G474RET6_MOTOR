/**
 * @file m1_flux_id_1000rpm.profile.h
 * @date 2026-10-06
 * @brief 磁链/Id 相关辨识 profile。

 *
 * 只铺宏。切实验改 bringup_active.h。
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
 */

#ifndef CONFIG_PROFILES_M1_FLUX_ID_1000RPM_PROFILE_H
#define CONFIG_PROFILES_M1_FLUX_ID_1000RPM_PROFILE_H

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

/** HOLD：settle + 斜坡到 1000 rpm（与旧 SPEED_IDENT 同结构，仅目标改 1000） */
#define M1_SPEED_IDENT_HOLD_S           8.0f
#ifndef M1_SPEED_IDENT_RPM_START
#define M1_SPEED_IDENT_RPM_START        100.0f
#endif
#define M1_SPEED_IDENT_STEP_ROUNDS      1u
#define M1_SPEED_IDENT_STEP_RPM0        1000.0f
#define M1_SPEED_IDENT_STEP_RPM1        1000.0f
#define M1_SPEED_IDENT_STEP_RPM2        1000.0f
#define M1_SPEED_IDENT_STEP_RPM3        1000.0f
#define M1_SPEED_IDENT_STEP_DWELL_S     10.0f
#define M1_SPEED_IDENT_STEP_ZERO_DWELL_S 10.0f

#ifndef M1_SPEED_IDENT_PLL_SETTLE_S
#define M1_SPEED_IDENT_PLL_SETTLE_S     1.5f
#endif

#undef M1_VOFA_PLL_CH8_11
#define M1_VOFA_PLL_CH8_11              0
#undef M1_VOFA_SPEED_CH8_11
#define M1_VOFA_SPEED_CH8_11            1

/** 与旧速度环一致：12ch；仅降采样到 4 kHz */
#undef M1_VOFA_UNIFIED_12CH
#define M1_VOFA_UNIFIED_12CH            1
#undef M1_TELEM_BRINGUP_K
#define M1_TELEM_BRINGUP_K              12u
#undef M1_TELEM_BRINGUP_DECIMATION
#define M1_TELEM_BRINGUP_DECIMATION     5u
#undef M1_VOFA_FLUX_ID_6CH
#define M1_VOFA_FLUX_ID_6CH             0
#undef M1_VOFA_CH11_ENC_RAW
#define M1_VOFA_CH11_ENC_RAW            1

#undef M1_CLOSURE_BRINGUP
#define M1_CLOSURE_BRINGUP              0

#endif /* CONFIG_PROFILES_M1_FLUX_ID_1000RPM_PROFILE_H */
