/**
 * @file bringup_active.h
 * @brief 联调实例选择 — 平时只改本文件。
 *
 * Bode / SPEED / FLUX / OBS_VEQ / IF：见各 profile。
 * HFI：config/profiles/m1_hfi_standstill.profile.h
 *   交付只认 M1_HFI_GATE=141（±1500 保持 0.5 s 换向 ×5）。
 *   1…140 旧关卡注释已废，勿改 GATE 回退。
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
