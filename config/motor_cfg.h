/**
 * @file motor_cfg.h
 * @date 2026-10-07
 * @brief 电机铭牌运行时只读表（包 8.3）。
 *
 * 与 `motor_params_m1.h` 宏并存：宏仍供编译期计算与 `#if`；
 * 算法热路径逐步改读本表。换电机改实例或将来 NVM，不改调用点。
 */

#ifndef CONFIG_MOTOR_CFG_H
#define CONFIG_MOTOR_CFG_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 电机铭牌（const / 将来可来自 NVM）。单位见成员注释。 */
typedef struct {
    /** 定子电阻，Ω。 */
    float rs_ohm;
    /** d 轴电感，H。 */
    float ld_h;
    /** q 轴电感，H。 */
    float lq_h;
    /** 母线电压标称，V（补偿/限幅用）。 */
    float vbus_v;
    /** 有效死区，ns（含驱动与 MOS；非定时器编程值）。 */
    uint32_t deadtime_ns;
    /** 极对数。 */
    uint8_t pole_pairs;
} motor_cfg_t;

/** M1 铭牌实例（由宏灌入，见 motor_cfg_m1.c）。 */
extern const motor_cfg_t g_m1_motor_cfg;

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_MOTOR_CFG_H */
