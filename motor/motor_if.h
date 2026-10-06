/**
 * @file motor_if.h
 * @date 2026-10-06
 * @brief 真 I-f：电流闭环加强制电角斜坡。
 *
 * tick 只允许从电流环节拍调用。release 之后不再驱动 θ 和 Iq。
 * 与 Ud/Uq 开环、Uq 启动互斥。
 *
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
 */

#ifndef MOTOR_IF_H
#define MOTOR_IF_H

#include "motor_context.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * I-f 状态。ALIGN 对中；RAMP 爬速；HOLD 停在目标速；STOP 正常停；FAULT 故障。
 */
typedef enum {
    /** 对中，电角不动。 */
    M1_IF_ALIGN = 0,
    /** 电角斜坡加速。 */
    M1_IF_RAMP,
    /** 停在目标转速。 */
    M1_IF_HOLD,
    /** 已停，不再驱动。 */
    M1_IF_STOP,
    /** 故障锁。 */
    M1_IF_FAULT
} m1_if_state_t;

typedef struct {
    /** 本拍状态。 */
    m1_if_state_t state;
    /** 强制 Park 电角，单位 rad。 */
    float theta_park;
    /** Iq 指令，单位 A。 */
    float iq_ref;
    /** Id 指令，单位 A。 */
    float id_ref;
    /** 当前强制转速指令，单位 rpm。 */
    float omega_cmd_rpm;
    /** 对照测得转速，单位 rpm。 */
    float omega_meas_rpm;
    /** wrap(θ_if − θ_enc)，单位 rad。 */
    float theta_err_rad;
} motor_if_step_t;

void motor_if_init(motor_context_t *ctx);
void motor_if_arm(motor_context_t *ctx);
void motor_if_release(void);
uint8_t motor_if_is_driving(void);
void motor_if_set_target_rpm(float rpm);
float motor_if_get_target_rpm(void);
motor_if_step_t motor_if_tick(motor_context_t *ctx, float theta_enc_park);
m1_if_state_t motor_if_get_state(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_IF_H */
