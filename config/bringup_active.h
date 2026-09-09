/**
 * @file bringup_active.h
 * @brief 联调实例选择 — 平时只改本文件。
 *
 * Bode 扫频配方（fc / 激励 / 频表）在：
 *   config/profiles/m1_bode_id_fc1000.profile.h
 *
 * 用法：下面只保留一行有效的 M1_BRINGUP_MODE 定义。
 */
#ifndef CONFIG_BRINGUP_ACTIVE_H
#define CONFIG_BRINGUP_ACTIVE_H

#undef M1_BRINGUP_MODE

/* ---- 选一个 ---- */
/* #define M1_BRINGUP_MODE  M1_BRINGUP_MODE_NORMAL */
#define M1_BRINGUP_MODE  M1_BRINGUP_MODE_BODE_ID_OFF_ONLY   /* Id Bode 签收 fc=1000 */
/* #define M1_BRINGUP_MODE  M1_BRINGUP_MODE_BODE_OFF_ONLY */ /* Iq Bode */
/* #define M1_BRINGUP_MODE  M1_BRINGUP_MODE_SPEED_IDENT */

#endif /* CONFIG_BRINGUP_ACTIVE_H */
