/**
 * @file bringup_active.h
 * @date 2026-10-06
 * @brief 联调实例选择。平时只改本文件。
 *
 * 改两处即可：
 *   1) M1_ACTIVE_ROUTE_ID — 选哪张 route 表（USE_* profile 由 params 按 id 推导）
 *   2) M1_BRINGUP_MODE — 标定/辨识配方（PASS0_RS 等）
 * 现网：route=9 (r_id_cal_pass0_rs) + PASS0_RS 标定链。
 *
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
 */
#ifndef CONFIG_BRINGUP_ACTIVE_H
#define CONFIG_BRINGUP_ACTIVE_H

#include "route_ids.h"

/* ---- 选路（与 config/routes/r_*.c 的 route_id 对齐；1 与 9 已进链接） ---- */
#undef M1_ACTIVE_ROUTE_ID
#define M1_ACTIVE_ROUTE_ID  M1_ROUTE_ID_ID_CAL_PASS0_RS

#undef M1_BRINGUP_MODE
#define M1_BRINGUP_MODE  M1_BRINGUP_MODE_ID_CAL_PASS0_RS

#undef M1_EXP_FRAMEWORK_ENABLE
#define M1_EXP_FRAMEWORK_ENABLE         1
#undef M1_EXP_BOOT_CALIB_CHAIN
#define M1_EXP_BOOT_CALIB_CHAIN         1
#undef M1_EXP_BOOT_UD_CAL
#define M1_EXP_BOOT_UD_CAL              0
#undef M1_EXP_BOOT_REV141
#define M1_EXP_BOOT_REV141              0

#endif /* CONFIG_BRINGUP_ACTIVE_H */
