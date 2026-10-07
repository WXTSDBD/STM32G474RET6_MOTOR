/**
 * @file exp_runner.h
 * @date 2026-10-07
 * @brief 实验执行器：选实验、武装、外环 tick（E0）。
 *
 * 只在 2 kHz 外环路径调用 exp_runner_tick。不要从 20 kHz 电流环直接调。
 */

#ifndef MOTOR_EXPERIMENT_EXP_RUNNER_H
#define MOTOR_EXPERIMENT_EXP_RUNNER_H

#include <stdint.h>

#include "exp_def.h"

#ifdef __cplusplus
extern "C" {
#endif

void exp_runner_init(void);
uint8_t exp_runner_select(uint8_t exp_id);
void exp_runner_arm(motor_context_t *ctx);
void exp_runner_tick(motor_context_t *ctx, float theta_fb_rad, float omega_rpm);
uint8_t exp_runner_is_armed(void);
uint8_t exp_runner_active_id(void);
const exp_def_t *exp_runner_active(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_EXPERIMENT_EXP_RUNNER_H */
