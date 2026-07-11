/**
 * @file motor_pll.h
 * @brief 机械角域 Type-II PLL：编码器 unwrap 角 → 平滑 omega_mech。
 */

#ifndef MOTOR_PLL_H
#define MOTOR_PLL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float theta;              /* PLL 跟踪角 [rad] mech，连续多圈 */
    float omega;              /* 估计机械角速度 [rad/s] */
    float integrator;         /* PI 积分项 → omega 偏置 */
    float kp;
    float ki;
    float omega_limit;        /* |omega| 上限 [rad/s] */
    float integrator_limit;   /* |integrator| 上限 [rad/s] */
    float last_err;           /* 最近一次相位误差 [-π, π] */
} motor_pll_t;

void motor_pll_init(motor_pll_t *pll,
                    float kp,
                    float ki,
                    float omega_limit,
                    float integrator_limit);

void motor_pll_reset(motor_pll_t *pll, float theta0);

void motor_pll_update(motor_pll_t *pll, float theta_meas, float dt);

float motor_pll_get_theta(const motor_pll_t *pll);
float motor_pll_get_omega_mech(const motor_pll_t *pll);
float motor_pll_get_omega_mech_rpm(const motor_pll_t *pll);
float motor_pll_get_last_err(const motor_pll_t *pll);

/** pole_pairs × theta_mech + offset，wrap 到 [0, 2π)。 */
float motor_pll_theta_el(const motor_pll_t *pll,
                         uint8_t pole_pairs,
                         float offset_rad);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_PLL_H */
