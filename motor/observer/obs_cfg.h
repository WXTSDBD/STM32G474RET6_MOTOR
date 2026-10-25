/**
 * @file obs_cfg.h
 * @date 2026-10-07
 * @brief 观测器算法配置入口。不含函数原型。
 *
 * 算法 .c 应走 OBS_* 契约名，不要再直接 include motor_params_m1.h。
 * 铭牌运行时真相源是 `g_m1_motor_cfg`（init / 慢路径优先读表）。
 * OBS_* 宏仍等于 M1_*，供编译期常量与未迁热路径；节拍仍走宏。
 * 发布门槛默认写在本头；profile 与默认同值时不必再复写。
 *
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
 */

#ifndef MOTOR_OBSERVER_OBS_CFG_H
#define MOTOR_OBSERVER_OBS_CFG_H

#include <stdint.h>

#include "motor_cfg.h"
#include "motor_params_m1.h"

/** 定子电阻，单位 ohm。编译期默认；运行时优先 g_m1_motor_cfg.rs_ohm。 */
#ifndef OBS_RS_OHM
#define OBS_RS_OHM              M1_RS_OHM
#endif
/** d 轴电感，单位 H。 */
#ifndef OBS_LD_H
#define OBS_LD_H                M1_LD_H
#endif
/** q 轴电感，单位 H。 */
#ifndef OBS_LQ_H
#define OBS_LQ_H                M1_LQ_H
#endif
/** 极对数。电角与机械转速换算都靠它。 */
#ifndef OBS_POLE_PAIRS
#define OBS_POLE_PAIRS          M1_POLE_PAIRS
#endif
/** 控制节拍，单位 s。默认 50 us。 */
#ifndef OBS_CTRL_TS_S
#define OBS_CTRL_TS_S           M1_CTRL_TS_S
#endif

/**
 * 发布门槛，单位机械 rpm。默认；profile 可覆盖。
 * UP：未发布时用低速 ω 判断何时切到 SMO。
 * DOWN：已发布时用 SMO ω 判断何时退回 HFI。不要用卡住的 HFI 积分做退门。
 * INJ_OFF / INJ_ON：注入关断与重新打开的回滞。
 */
#ifndef M1_HFI_PUB_UP_RPM
#define M1_HFI_PUB_UP_RPM       1300.0f
#endif
#ifndef M1_HFI_PUB_DOWN_RPM
#define M1_HFI_PUB_DOWN_RPM     1000.0f
#endif
#ifndef M1_HFI_PUB_INJ_OFF_RPM
#define M1_HFI_PUB_INJ_OFF_RPM  1500.0f
#endif
#ifndef M1_HFI_PUB_INJ_ON_RPM
#define M1_HFI_PUB_INJ_ON_RPM   1400.0f
#endif
/** 退回 HFI：|ω_smo|≤DOWN 连续满足的拍数（20 kHz）。 */
#ifndef M1_HFI_PUB_DN_QUAL_N
#define M1_HFI_PUB_DN_QUAL_N    400u
#endif
/**
 * 1=SMO 在中速带「转速冻住」则强制回 HFI。
 * 假速钉死时 |ω_smo| 可能永远高于 DOWN，单靠 DOWN 退不回来。
 */
#ifndef M1_HFI_PUB_FREEZE_ENABLE
#define M1_HFI_PUB_FREEZE_ENABLE 1
#endif
#ifndef M1_HFI_PUB_FREEZE_RPM_MIN
#define M1_HFI_PUB_FREEZE_RPM_MIN 200.0f
#endif
#ifndef M1_HFI_PUB_FREEZE_RPM_MAX
#define M1_HFI_PUB_FREEZE_RPM_MAX 900.0f
#endif
/** 窗内 |Δrpm| 小于此值视为冻住（对照未滤波 ω）。 */
#ifndef M1_HFI_PUB_FREEZE_DRPM
#define M1_HFI_PUB_FREEZE_DRPM  15.0f
#endif
#ifndef M1_HFI_PUB_FREEZE_N
#define M1_HFI_PUB_FREEZE_N     400u
#endif

#endif /* MOTOR_OBSERVER_OBS_CFG_H */
