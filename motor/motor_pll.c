/**
 * @file motor_pll.c
 * @date 2026-10-06
 * @brief 机械角 Type-II PLL：相位误差 PI 得到 ω，再积分 θ。
 *
 * 节拍限制见 motor_pll.h 文件头。
 *
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
 */

#include "motor_pll.h"

#include <stddef.h>

#ifndef MOTOR_PLL_TWO_PI
#define MOTOR_PLL_TWO_PI 6.28318530718f
#endif

#ifndef MOTOR_PLL_PI
#define MOTOR_PLL_PI 3.14159265359f
#endif

/**
 * @brief 折到 (-π, π]。
 */
static float motor_pll_wrap_pi(float rad)
{
    while (rad > MOTOR_PLL_PI) {
        rad -= MOTOR_PLL_TWO_PI;
    }
    while (rad < -MOTOR_PLL_PI) {
        rad += MOTOR_PLL_TWO_PI;
    }
    return rad;
}

/**
 * @brief 折到 [0, 2π)。
 */
static float motor_pll_wrap_0_2pi(float rad)
{
    while (rad >= MOTOR_PLL_TWO_PI) {
        rad -= MOTOR_PLL_TWO_PI;
    }
    while (rad < 0.0f) {
        rad += MOTOR_PLL_TWO_PI;
    }
    return rad;
}

/**
 * @brief 按绝对值上限截断。
 */
static float motor_pll_clamp(float x, float limit)
{
    if (x > limit) {
        return limit;
    }
    if (x < -limit) {
        return -limit;
    }
    return x;
}

/**
 * @brief 写入增益和限幅并清状态。
 * @param pll PLL。不可为 NULL。
 * @param kp 比例。
 * @param ki 积分。
 * @param omega_limit |ω| 上限，单位 rad/s。
 * @param integrator_limit |积分| 上限，单位 rad/s。
 */
void motor_pll_init(motor_pll_t *pll,
                    float kp,
                    float ki,
                    float omega_limit,
                    float integrator_limit)
{
    if (pll == NULL) {
        return;
    }

    pll->theta = 0.0f;
    pll->omega = 0.0f;
    pll->integrator = 0.0f;
    pll->kp = kp;
    pll->ki = ki;
    pll->omega_limit = omega_limit;
    pll->integrator_limit = integrator_limit;
    pll->last_err = 0.0f;
}

/**
 * @brief 把跟踪角放到 theta0，清 ω 和积分。
 * @param pll PLL。不可为 NULL。
 * @param theta0 机械角，单位 rad。
 */
void motor_pll_reset(motor_pll_t *pll, float theta0)
{
    if (pll == NULL) {
        return;
    }

    pll->theta = theta0;
    pll->omega = 0.0f;
    pll->integrator = 0.0f;
    pll->last_err = 0.0f;
}

/**
 * @brief 用测量机械角推进一步。
 * @param pll PLL。不可为 NULL。
 * @param theta_meas 测量机械角，单位 rad。
 * @param dt 节拍，单位 s。
 */
void motor_pll_update(motor_pll_t *pll, float theta_meas, float dt)
{
    float err;
    float omega_cmd;

    if (pll == NULL || dt <= 0.0f) {
        return;
    }

    err = motor_pll_wrap_pi(theta_meas - pll->theta);
    pll->last_err = err;

    pll->integrator += pll->ki * err * dt;
    pll->integrator = motor_pll_clamp(pll->integrator, pll->integrator_limit);

    omega_cmd = pll->integrator + pll->kp * err;
    if (omega_cmd > pll->omega_limit) {
        pll->omega = pll->omega_limit;
        pll->integrator = pll->omega_limit - pll->kp * err;
        pll->integrator = motor_pll_clamp(pll->integrator, pll->integrator_limit);
    } else if (omega_cmd < -pll->omega_limit) {
        pll->omega = -pll->omega_limit;
        pll->integrator = -pll->omega_limit - pll->kp * err;
        pll->integrator = motor_pll_clamp(pll->integrator, pll->integrator_limit);
    } else {
        pll->omega = omega_cmd;
    }

    pll->theta += pll->omega * dt;
}

/**
 * @brief 读跟踪机械角，单位 rad。pll 为 NULL 时返回 0。
 */
float motor_pll_get_theta(const motor_pll_t *pll)
{
    if (pll == NULL) {
        return 0.0f;
    }
    return pll->theta;
}

float motor_pll_get_omega_mech(const motor_pll_t *pll)
{
    if (pll == NULL) {
        return 0.0f;
    }
    return pll->omega;
}

/**
 * @brief 读机械转速，单位 rpm。
 */
float motor_pll_get_omega_mech_rpm(const motor_pll_t *pll)
{
    if (pll == NULL) {
        return 0.0f;
    }
    return pll->omega * 60.0f / MOTOR_PLL_TWO_PI;
}

/**
 * @brief 读最近一次相位误差，单位 rad。
 */
float motor_pll_get_last_err(const motor_pll_t *pll)
{
    if (pll == NULL) {
        return 0.0f;
    }
    return pll->last_err;
}

float motor_pll_theta_el(const motor_pll_t *pll,
                         uint8_t pole_pairs,
                         float offset_rad)
{
    float theta_el;

    if (pll == NULL || pole_pairs == 0u) {
        return 0.0f;
    }

    theta_el = pll->theta * (float)pole_pairs + offset_rad;
    return motor_pll_wrap_0_2pi(theta_el);
}
