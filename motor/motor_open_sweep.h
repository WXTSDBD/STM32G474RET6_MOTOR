/**
 * @file motor_open_sweep.h
 * @brief 开环 Uq 可插拔：V0/V1/V2 扫参、AB 死区对比、固定 Uq 定时。
 */

#ifndef MOTOR_OPEN_SWEEP_H
#define MOTOR_OPEN_SWEEP_H

#include "motor_context.h"

#ifdef __cplusplus
extern "C" {
#endif

void motor_open_sweep_init(motor_context_t *ctx);

/** 上电 OBSERVE/OPEN 模式：2/2.5/3 V 或 AB 六段扫参。 */
void motor_open_sweep_arm_v012(motor_context_t *ctx);

/** 固定 Uq（V）运行 duration_s 秒；调用后需 ctx->mode=OPEN/OBSERVE。 */
void motor_open_sweep_begin_fixed(motor_context_t *ctx, float uq_v, float duration_s);

/** OPEN/OBSERVE 模式下每 JEOC 调用，更新 ctx->uq_open。 */
void motor_open_sweep_tick(motor_context_t *ctx);

uint8_t motor_open_sweep_active(void);
uint8_t motor_open_sweep_done(void);

#ifdef __cplusplus
}
#endif

#endif
