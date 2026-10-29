/**
 * @file bringup_active.h
 * @date 2026-10-06
 * @brief 联调实例选择。平时只改本文件。
 *
 * 改两处即可：
 *   1) M1_ACTIVE_ROUTE_ID — 选哪张 route 表（USE_* profile 由 params 按 id 推导）
 *   2) M1_BRINGUP_MODE — 标定/辨识配方（PASS0_RS / SPEED_IDENT 等）
 *
 * 本包：route=1 (hfi_standstill) + SPEED_IDENT + EXP_BOOT_REV141
 *       → 上电跑 ±2000 无感两对 + 刹停 + 有感一对（金样路径）。
 * 回 PASS0：route=9 + ID_CAL_PASS0_RS + EXP_BOOT_CALIB_CHAIN=1 + REV141=0。
 *
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
 */
#ifndef CONFIG_BRINGUP_ACTIVE_H
#define CONFIG_BRINGUP_ACTIVE_H

#include "route_ids.h"

/* ---- 选路（与 config/routes/r_*.c 的 route_id 对齐） ---- */
#undef M1_ACTIVE_ROUTE_ID
#define M1_ACTIVE_ROUTE_ID  M1_ROUTE_ID_HFI_STANDSTILL

#undef M1_BRINGUP_MODE
#define M1_BRINGUP_MODE  M1_BRINGUP_MODE_SPEED_IDENT

#undef M1_EXP_FRAMEWORK_ENABLE
#define M1_EXP_FRAMEWORK_ENABLE         1
#undef M1_EXP_BOOT_CALIB_CHAIN
#define M1_EXP_BOOT_CALIB_CHAIN         0
#undef M1_EXP_BOOT_UD_CAL
#define M1_EXP_BOOT_UD_CAL              0
#undef M1_EXP_BOOT_REV141
#define M1_EXP_BOOT_REV141              1

#endif /* CONFIG_BRINGUP_ACTIVE_H */
