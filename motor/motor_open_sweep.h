/**
 * @file motor_open_sweep.h
 * @date 2026-10-06
 * @brief 开环 Ud/Uq 扫参（V012 / 固定 Uq）。预 Id 阶梯已下线。
 *
 * tick 只允许从电流环节拍、开环或观察模式调用。
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
 */

#ifndef MOTOR_OPEN_SWEEP_H
#define MOTOR_OPEN_SWEEP_H

#include "motor_context.h"

#ifdef __cplusplus
extern "C" {
#endif

void motor_open_sweep_init(motor_context_t *ctx);
void motor_open_sweep_arm_v012(motor_context_t *ctx);
void motor_open_sweep_begin_fixed(motor_context_t *ctx, float uq_v, float duration_s);
void motor_open_sweep_tick(motor_context_t *ctx);
uint8_t motor_open_sweep_active(void);
uint8_t motor_open_sweep_done(void);

static inline uint8_t motor_open_sweep_use_fix_theta(void) { return 0u; }
static inline uint8_t motor_open_sweep_in_pre_id_align(void) { return 0u; }
static inline uint8_t motor_open_sweep_in_pre_id_ud_ladder(void) { return 0u; }

#ifdef __cplusplus
}
#endif

#endif
