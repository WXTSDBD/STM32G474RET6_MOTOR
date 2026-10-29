/**
 * @file m1_n5065.h
 * @date 2026-10-28
 * @brief N5065 电机铭牌（plant）对外声明。
 *
 * 数值与 8 份 profile 共用的 M1_RS/LD/LQ/POLE/VBUS/DEADTIME 同源。
 * route 经 g_plant_m1_n5065 引用；勿在 r_*.c 再复制一份 plant。
 */
#ifndef CONFIG_PLANT_M1_N5065_H
#define CONFIG_PLANT_M1_N5065_H

#include "core/motor_types.h"

#ifdef __cplusplus
extern "C" {
#endif

extern const motor_plant_t g_plant_m1_n5065;

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_PLANT_M1_N5065_H */
