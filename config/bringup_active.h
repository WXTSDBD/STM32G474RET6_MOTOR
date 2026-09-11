/**
 * @file bringup_active.h
 * @brief 联调实例选择 — 平时只改本文件。
 *
 * Bode：config/profiles/m1_bode_id_fc1000.profile.h
 * 速度 1000 rpm：config/profiles/m1_speed_1000rpm.profile.h（须 SPEED_IDENT）
 * 磁链稳速：config/profiles/m1_flux_id_1000rpm.profile.h（须 SPEED_IDENT）
 * Veq 旁路：config/profiles/m1_obs_veq_1000rpm.profile.h（须 SPEED_IDENT）
 *
 * 用法：下面只保留一行有效的 M1_BRINGUP_MODE；SPEED_1000 / FLUX / OBS_VEQ 互斥。
 */
#ifndef CONFIG_BRINGUP_ACTIVE_H
#define CONFIG_BRINGUP_ACTIVE_H

#undef M1_BRINGUP_MODE

/* ---- 选一个模式 ---- */
/* #define M1_BRINGUP_MODE  M1_BRINGUP_MODE_NORMAL */
/* #define M1_BRINGUP_MODE  M1_BRINGUP_MODE_BODE_ID_OFF_ONLY */
/* #define M1_BRINGUP_MODE  M1_BRINGUP_MODE_BODE_OFF_ONLY */
#define M1_BRINGUP_MODE  M1_BRINGUP_MODE_SPEED_IDENT

/*
 * SPEED_1000：有感阶梯 + SMO→EMF-PLL 旁路（无感 SMO 实验）
 * FLUX：有感 1000 rpm 估 ψf
 * OBS_VEQ：有感 Veq/SMO 旁路 + 阶梯（另一套 profile）
 * 仅 SPEED_IDENT 下有效；三者勿同时为 1
 */
#undef M1_USE_SPEED_1000_PROFILE
#define M1_USE_SPEED_1000_PROFILE       1
#undef M1_USE_FLUX_ID_PROFILE
#define M1_USE_FLUX_ID_PROFILE          0
#undef M1_USE_OBS_VEQ_PROFILE
#define M1_USE_OBS_VEQ_PROFILE          0

#endif /* CONFIG_BRINGUP_ACTIVE_H */
