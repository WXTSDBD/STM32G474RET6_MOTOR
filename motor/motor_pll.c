/**
 * @file motor_pll.c
 * @brief 机械角域 Type-II PLL（相位误差 PI → ω → ∫θ）。
 */

#include "motor_pll.h"

#include <stddef.h>

#ifndef MOTOR_PLL_TWO_PI
#define MOTOR_PLL_TWO_PI 6.28318530718f
#endif

#ifndef MOTOR_PLL_PI
#define MOTOR_PLL_PI 3.14159265359f
#endif

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

float motor_pll_get_omega_mech_rpm(const motor_pll_t *pll)
{
    if (pll == NULL) {
        return 0.0f;
    }
    return pll->omega * 60.0f / MOTOR_PLL_TWO_PI;
}

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
