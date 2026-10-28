/**
 * @file bringup_active.h
 * @date 2026-10-06
 * @brief 联调实例选择。平时只改本文件。
 *
 * 2026-10-28：PASS0_RS = ALIGN → Rs + VESC HF-L×10（仅 l_hfi，无 VASI）；EXP_ID_CALIB_CHAIN。
 * VOFA：6 通道 @ 20 kHz（id iq ud uq flex mark）；layout=IDENT_DUTY 紧凑表。
 *
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
 */
#ifndef CONFIG_BRINGUP_ACTIVE_H
#define CONFIG_BRINGUP_ACTIVE_H

#undef M1_BRINGUP_MODE
#define M1_BRINGUP_MODE  M1_BRINGUP_MODE_ID_CAL_PASS0_RS

/* 不上 ±2000 HFI 交付 profile；HFI 数值见 PASS0_RS 内 m1_hfi_compile_nums.h。 */
#undef M1_USE_HFI_STANDSTILL_PROFILE
#define M1_USE_HFI_STANDSTILL_PROFILE   0
#undef M1_USE_SENSED_POS_MIT_PROFILE
#define M1_USE_SENSED_POS_MIT_PROFILE   0
#undef M1_USE_SENSORLESS_POS_MIT_PROFILE
#define M1_USE_SENSORLESS_POS_MIT_PROFILE 0
#undef M1_USE_IF_100_PROFILE
#define M1_USE_IF_100_PROFILE           0
#undef M1_USE_SPEED_1000_PROFILE
#define M1_USE_SPEED_1000_PROFILE       0
#undef M1_USE_FLUX_ID_PROFILE
#define M1_USE_FLUX_ID_PROFILE          0
#undef M1_USE_OBS_VEQ_PROFILE
#define M1_USE_OBS_VEQ_PROFILE          0

#undef M1_EXP_FRAMEWORK_ENABLE
#define M1_EXP_FRAMEWORK_ENABLE         1
#undef M1_EXP_BOOT_CALIB_CHAIN
#define M1_EXP_BOOT_CALIB_CHAIN         1
#undef M1_EXP_BOOT_UD_CAL
#define M1_EXP_BOOT_UD_CAL              0
#undef M1_EXP_BOOT_REV141
#define M1_EXP_BOOT_REV141              0

#endif /* CONFIG_BRINGUP_ACTIVE_H */
