/**
 * @file obs_cfg.h
 * @date 2026-10-06
 * @brief 观测器算法配置入口。不含函数原型。
 *
 * 算法 .c 应走 OBS_* 契约名，不要再直接 include motor_params_m1.h。
 * 电机差（电阻、电感、极对数、节拍）从这里进来。发布门槛默认写在本头；
 * profile 与默认同值时不必再复写。
 *
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
 */

#ifndef MOTOR_OBSERVER_OBS_CFG_H
#define MOTOR_OBSERVER_OBS_CFG_H

#include <stdint.h>
#include "motor_params_m1.h"

/** 定子电阻，单位 ohm。默认等于电机参数。 */
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

#endif /* MOTOR_OBSERVER_OBS_CFG_H */
