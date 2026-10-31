/**
 * @file obs_cfg.h
 * @date 2026-10-07
 * @brief 观测器算法配置入口。不含函数原型。
 *
 * 算法 .c 应走 OBS_* 契约名，不要再直接 include motor_params_m1.h。
 * 铭牌运行时真相源是 `g_m1_motor_cfg`（init / 慢路径优先读表）。
 * OBS_* 宏仍等于 M1_*，供编译期常量与未迁热路径；节拍仍走宏。
 * 发布门槛、踢段门槛、SMO/PLL/电压方程系数默认在 defaults_obs.h；
 * recipe 用 #undef+#define 覆盖。本头只做 OBS_* 别名。
 *
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
 */

#ifndef MOTOR_OBSERVER_OBS_CFG_H
#define MOTOR_OBSERVER_OBS_CFG_H

#include <stdint.h>

#include "motor_cfg.h"
#include "motor_params_m1.h"

/** 定子电阻，单位 ohm。编译期默认；运行时优先 g_m1_motor_cfg.rs_ohm。 */
#define OBS_RS_OHM              M1_RS_OHM
/** d 轴电感，单位 H。 */
#define OBS_LD_H                M1_LD_H
/** q 轴电感，单位 H。 */
#define OBS_LQ_H                M1_LQ_H
/** 极对数。电角与机械转速换算都靠它。 */
#define OBS_POLE_PAIRS          M1_POLE_PAIRS
/** 控制节拍，单位 s。默认 50 us。 */
#define OBS_CTRL_TS_S           M1_CTRL_TS_S

#endif /* MOTOR_OBSERVER_OBS_CFG_H */
