/**
 * @file motor_if.h
 * @brief 真 I/F：电流闭环 + 强制电角度斜坡（与 Ud/Uq 开环、Uq startup 互斥）
 *
 * M1_IF_TO_OBS：爬到目标后可由软切接走；motor_if_release() 后不再驱动 θ/Iq。
 */
#ifndef MOTOR_IF_H
#define MOTOR_IF_H

#include "motor_context.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    M1_IF_ALIGN = 0,
    M1_IF_RAMP,
    M1_IF_HOLD,
    M1_IF_STOP,
    M1_IF_FAULT
} m1_if_state_t;

typedef struct {
    m1_if_state_t state;
    float theta_park;
    float iq_ref;
    float id_ref;
    float omega_cmd_rpm;
    float omega_meas_rpm;
    float theta_err_rad;
} motor_if_step_t;

void motor_if_init(motor_context_t *ctx);
void motor_if_arm(motor_context_t *ctx);

/** 交给无感后调用：停止强制 θ/Iq */
void motor_if_release(void);

/** 1=仍在驱动 Park/Iq（未 release 且非 STOP/FAULT） */
uint8_t motor_if_is_driving(void);

/**
 * 运行时 I/F 目标转速 [rpm]（含符号）。
 * @note 正→停→反序列用：零速后再 arm 前改成 −|ω|；默认 = M1_IF_TARGET_RPM。
 */
void motor_if_set_target_rpm(float rpm);
float motor_if_get_target_rpm(void);

motor_if_step_t motor_if_tick(motor_context_t *ctx, float theta_enc_park);

m1_if_state_t motor_if_get_state(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_IF_H */
