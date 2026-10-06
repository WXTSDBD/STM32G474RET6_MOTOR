/**
 * @file bringup_active.h
 * @date 2026-10-06
 * @brief 联调实例选择。平时只改本文件。
 *
 * 无感静止起动打开 M1_USE_HFI_STANDSTILL_PROFILE。切回有感把它置 0，再打开对应有感 profile。
 * 本文件只选 profile，不含函数。
 *
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
 */
#ifndef CONFIG_BRINGUP_ACTIVE_H
#define CONFIG_BRINGUP_ACTIVE_H

#undef M1_BRINGUP_MODE
#define M1_BRINGUP_MODE  M1_BRINGUP_MODE_SPEED_IDENT

#undef M1_USE_HFI_STANDSTILL_PROFILE
#define M1_USE_HFI_STANDSTILL_PROFILE   1
#undef M1_USE_IF_100_PROFILE
#define M1_USE_IF_100_PROFILE           0
#undef M1_USE_SPEED_1000_PROFILE
#define M1_USE_SPEED_1000_PROFILE       0
#undef M1_USE_FLUX_ID_PROFILE
#define M1_USE_FLUX_ID_PROFILE          0
#undef M1_USE_OBS_VEQ_PROFILE
#define M1_USE_OBS_VEQ_PROFILE          0

/*
 * 交付 GATE=141。旧 1…140 关卡表已废。
 */
#undef M1_HFI_GATE
#define M1_HFI_GATE                     141

#endif /* CONFIG_BRINGUP_ACTIVE_H */
