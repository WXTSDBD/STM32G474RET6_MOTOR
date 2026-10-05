/**
 * @file obs_cfg.h
 * @brief P9 / F6：观测器算法的唯一配置入口。
 *
 * 算法 .c 禁止再直接 #include "motor_params_m1.h"。
 * 电机差（Rs/L/极对数/Ts）走 OBS_* 契约名；GATE 仍编译期。
 * 本步不改数值。hfi_sqwave.c 已改用 OBS_POLE_PAIRS / OBS_CTRL_TS_S。
 * obs_cfg 仍 include motor_params（真剥头是后续刀）。
 */
#ifndef MOTOR_OBSERVER_OBS_CFG_H
#define MOTOR_OBSERVER_OBS_CFG_H

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

#endif /* MOTOR_OBSERVER_OBS_CFG_H */
