/**
 * @file obs_cfg.h
 * @brief 观测器算法配置入口（宏 / OBS_* 契约）。不含函数原型。
 *
 * 算法 .c 禁止再直接 #include "motor_params_m1.h"。
 * 电机差（Rs/L/极对数/Ts）走 OBS_* 契约名；GATE 仍编译期。
 * 实装仍 include motor_params_m1.h（真剥头是后续刀）。
 * PUB 门槛默认在此；141 profile 与默认同值时不再复写。
 */
#ifndef MOTOR_OBSERVER_OBS_CFG_H
#define MOTOR_OBSERVER_OBS_CFG_H

#include <stdint.h>
#include "motor_params_m1.h"

#ifndef OBS_RS_OHM
#define OBS_RS_OHM              M1_RS_OHM
#endif
#ifndef OBS_LD_H
#define OBS_LD_H                M1_LD_H
#endif
#ifndef OBS_LQ_H
#define OBS_LQ_H                M1_LQ_H
#endif
#ifndef OBS_POLE_PAIRS
#define OBS_POLE_PAIRS          M1_POLE_PAIRS
#endif
#ifndef OBS_CTRL_TS_S
#define OBS_CTRL_TS_S           M1_CTRL_TS_S
#endif

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
