/**
 * @file bringup_active.h
 * @brief 联调实例选择 — 平时只改本文件。
 *
 * Bode：config/profiles/m1_bode_id_fc1000.profile.h
 * 速度 1000 rpm：config/profiles/m1_speed_1000rpm.profile.h（须 SPEED_IDENT）
 * 磁链稳速：config/profiles/m1_flux_id_1000rpm.profile.h（须 SPEED_IDENT）
 * Veq 旁路：config/profiles/m1_obs_veq_1000rpm.profile.h（须 SPEED_IDENT）
 * I/F 100 rpm：config/profiles/m1_if_100rpm.profile.h（须 SPEED_IDENT）
 *
 * 用法：下面只保留一行有效的 M1_BRINGUP_MODE；
 * SPEED_1000 / FLUX / OBS_VEQ / IF_100 互斥。
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
 * IF_100：真 I/F 拖到 100 rpm（无速度环阶梯 / 无 OBS / 无 Ud/Uq 开环）
 * SPEED_1000：有感阶梯 + SMO→EMF-PLL 软切
 * FLUX / OBS_VEQ：另见对应 profile
 * 仅 SPEED_IDENT 下有效；四者勿同时为 1
 */
#undef M1_USE_IF_100_PROFILE
#define M1_USE_IF_100_PROFILE           1
#undef M1_USE_SPEED_1000_PROFILE
#define M1_USE_SPEED_1000_PROFILE       0
#undef M1_USE_FLUX_ID_PROFILE
#define M1_USE_FLUX_ID_PROFILE          0
#undef M1_USE_OBS_VEQ_PROFILE
#define M1_USE_OBS_VEQ_PROFILE          0

#endif /* CONFIG_BRINGUP_ACTIVE_H */
