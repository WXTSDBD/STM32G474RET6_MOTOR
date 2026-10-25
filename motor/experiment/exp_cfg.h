/**
 * @file exp_cfg.h
 * @date 2026-10-09
 * @brief 实验 cfg → ctx.atrb 应用（E3：同电 A/B）。
 *
 * 只改运行时开关位，不改热路径结构。段表连续两段只差 cfg_idx 时由 runner 调用。
 */

#ifndef MOTOR_EXPERIMENT_EXP_CFG_H
#define MOTOR_EXPERIMENT_EXP_CFG_H

#include "exp_def.h"

#ifdef __cplusplus
extern "C" {
#endif

void exp_cfg_apply(motor_context_t *ctx, const exp_cfg_t *cfg);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_EXPERIMENT_EXP_CFG_H */
