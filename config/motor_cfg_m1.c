/**
 * @file motor_cfg_m1.c
 * @date 2026-10-07
 * @brief M1 铭牌实例：数值来自 motor_params_m1.h 宏。
 *
 * 本文件只灌表，不含控制逻辑。改铭牌默认值仍改宏（或将来 NVM）。
 */

#include "motor_cfg.h"

#include "motor_params_m1.h"

const motor_cfg_t g_m1_motor_cfg = {
    .rs_ohm = M1_RS_OHM,
    .ld_h = M1_LD_H,
    .lq_h = M1_LQ_H,
    .vbus_v = M1_VBUS_V,
    .deadtime_ns = (uint32_t)M1_DEADTIME_NS,
    .pole_pairs = (uint8_t)M1_POLE_PAIRS,
};
