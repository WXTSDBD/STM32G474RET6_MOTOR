/**
 * @file motor_startup.h
 * @brief M1 启动：ALIGN(Iq) → Uq 开环拖动 → 电流闭环。
 */

#ifndef MOTOR_STARTUP_H
#define MOTOR_STARTUP_H

#include "motor_context.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    M1_STARTUP_ALIGN = 0,
    M1_STARTUP_DRAG,
    M1_STARTUP_CLOSED,
    M1_STARTUP_FAULT,
} m1_startup_state_t;

typedef struct {
    m1_startup_state_t state;
    float theta_park;
    float iq_ref;
    float uq_out;
    float omega_mech_rpm;
    uint8_t use_fixed_uq;
    uint8_t pi_reset;
    uint8_t pi_bumpless;
    float uq_prev;
    float ud_prev;
} motor_startup_step_t;

void motor_startup_init(motor_context_t *ctx);
void motor_startup_arm(motor_context_t *ctx);

motor_startup_step_t motor_startup_tick(motor_context_t *ctx, float theta_enc_park);

/** Park 后调用：Iq 斜坡 + 切环 bumpless 标志（同拍） */
void motor_startup_finish_tick(motor_context_t *ctx, float iq_meas,
                               motor_startup_step_t *step);

m1_startup_state_t motor_startup_get_state(void);

#ifdef __cplusplus
}
#endif

#endif
