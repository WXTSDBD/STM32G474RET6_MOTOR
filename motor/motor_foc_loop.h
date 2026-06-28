/**
 * @file motor_foc_loop.h
 * @brief M1 Id/Iq PI 编排：init/reset、bumpless、Id cal / ident / 闭环分支。
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

/** deadband_flow_tick 之后：消费 ident PI 限幅重配请求。 */
void motor_foc_loop_on_flow_tick(motor_context_t *ctx);

/** 电流环模式下写 dbg.foc_id_ref。 */
void motor_foc_loop_dbg_id_ref(const motor_context_t *ctx);

/**
 * Park 后执行 PI：bumpless、Id cal / ident / 正常闭环、align Ud 覆盖；
 * 写 ctx->ud_pi/uq_pi 与 dbg.foc_ud_pi / dbg.foc_uq_pi；Id cal 采样同步。
 */
void motor_foc_loop_tick(motor_context_t *ctx,
                         float id,
                         float iq,
                         motor_startup_step_t *startup,
                         float theta_enc_park);

#ifdef __cplusplus
}
#endif

#endif
