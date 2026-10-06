/**
 * @file motor_foc_loop.h
 * @date 2026-10-06
 * @brief M1 Id/Iq PI 编排。算法在 foc_pi.c。
 *
 * tick 只允许从电流环节拍、Park 之后调用。标定和 ident 分支也在这里选。
 *
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
 */

#ifndef MOTOR_FOC_LOOP_H
#define MOTOR_FOC_LOOP_H

#include "motor_context.h"
#include "motor_startup.h"

#ifdef __cplusplus
extern "C" {
#endif

void motor_foc_loop_pi_init(motor_context_t *ctx);
void motor_foc_loop_pi_reset(motor_context_t *ctx);
void motor_foc_loop_on_flow_tick(motor_context_t *ctx);
void motor_foc_loop_dbg_id_ref(const motor_context_t *ctx);
void motor_foc_loop_tick(motor_context_t *ctx,
                         float id,
                         float iq,
                         motor_startup_step_t *startup,
                         float theta_enc_park);

#ifdef __cplusplus
}
#endif

#endif
