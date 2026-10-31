/**
 * @file bringup_active.h
 * @date 2026-10-06
 * @brief 联调实例选择。平时只改本文件。
 *
 * 改两处即可：
 *   1) M1_ACTIVE_ROUTE_ID — 选哪张 route 表
 *   2) M1_BOOT_EXP_ID — 上电实验（0=不跑；3=REV141；17=CALIB_CHAIN；见 exp_def.h）
 *
 * 本包：route=1 (hfi_standstill) + BOOT_EXP=REV141
 *       → 上电跑 ±2000 无感两对 + 刹停 + 有感一对。
 * 回 PASS0：route=9 + M1_BOOT_EXP_ID=17（EXP_ID_CALIB_CHAIN）。
 *
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
 */
#ifndef CONFIG_BRINGUP_ACTIVE_H
#define CONFIG_BRINGUP_ACTIVE_H

#include "route_ids.h"

/* ---- 选路（与 config/routes/r_*.c 的 route_id 对齐） ---- */
#undef M1_ACTIVE_ROUTE_ID
#define M1_ACTIVE_ROUTE_ID  M1_ROUTE_ID_HFI_STANDSTILL

#undef M1_EXP_FRAMEWORK_ENABLE
#define M1_EXP_FRAMEWORK_ENABLE         1
#undef M1_BOOT_EXP_ID
#define M1_BOOT_EXP_ID                  3u /* EXP_ID_REV141 */

#endif /* CONFIG_BRINGUP_ACTIVE_H */
