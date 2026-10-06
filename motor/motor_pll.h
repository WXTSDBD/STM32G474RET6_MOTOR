/**
 * @file motor_pll.h
 * @date 2026-10-06
 * @brief 机械角域 Type-II PLL：unwrap 角进，平滑机械转速出。
 *
 * update 只允许从电流环节拍调用。观测器速度反馈用的是另一份实例。
 *
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
 */

#ifndef MOTOR_PLL_H
#define MOTOR_PLL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    /** PLL 跟踪的机械角，单位 rad，连续多圈。 */
    float theta;
    /** 估计机械角速度，单位 rad/s。 */
    float omega;
    /** PI 积分，送到 ω。 */
    float integrator;
    /** 比例增益。 */
    float kp;
    /** 积分增益。 */
    float ki;
    /** |ω| 上限，单位 rad/s。 */
    float omega_limit;
    /** |积分| 上限，单位 rad/s。 */
    float integrator_limit;
    /** 最近一次相位误差，折到 (-π, π]。 */
    float last_err;
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
float motor_pll_theta_el(const motor_pll_t *pll,
                         uint8_t pole_pairs,
                         float offset_rad);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_PLL_H */
