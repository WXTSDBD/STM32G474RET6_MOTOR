/**
 * @file motor_current_exp_hooks.h
 * @date 2026-10-19
 * @brief 电流环实验旁路入口：辨识/开环阶梯等 OFF 宏时空桩。
 *
 * 热路径只调本头函数，避免 motor_current.c 堆实验嵌套。
 * 节拍限制与 motor_current.h 相同：只允许从电流环节拍调用。
 */

#ifndef MOTOR_CURRENT_EXP_HOOKS_H
#define MOTOR_CURRENT_EXP_HOOKS_H

#include "motor_context.h"

#ifdef __cplusplus
extern "C" {
#endif

void motor_current_exp_flow_tick(motor_context_t *ctx);

uint8_t motor_current_exp_try_ld_lq_udq(motor_context_t *ctx, float id,
                                       float iq, float *ud, float *uq);

uint8_t motor_current_exp_try_open_udq(motor_context_t *ctx, float *ud,
                                      float *uq);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_CURRENT_EXP_HOOKS_H */
