/**
 * @file motor_startup.h
 * @date 2026-10-06
 * @brief Uq 开环拖动再切电流闭环。
 *
 * tick 和 finish_tick 只允许从电流环节拍调用。finish_tick 必须在 Park 之后、同拍。
 *
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
 */

#ifndef MOTOR_STARTUP_H
#define MOTOR_STARTUP_H

#include "motor_context.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * 开环启动状态。ALIGN 对中；DRAG 开环拖；CLOSED 已切闭环；FAULT 故障。
 */
typedef enum {
    /** Iq 对中。 */
    M1_STARTUP_ALIGN = 0,
    /** Uq 开环拖动。 */
    M1_STARTUP_DRAG,
    /** 已切电流闭环。 */
    M1_STARTUP_CLOSED,
    /** 故障锁。 */
    M1_STARTUP_FAULT,
} m1_startup_state_t;

typedef struct {
    /** 本拍状态。 */
    m1_startup_state_t state;
    /** Park 电角，单位 rad。 */
    float theta_park;
    /** Iq 指令，单位 A。 */
    float iq_ref;
    /** 开环 Uq，单位 V。 */
    float uq_out;
    /** 对照机械转速，单位 rpm。 */
    float omega_mech_rpm;
    /** 1=本拍用固定 Uq，不吃 PI。 */
    uint8_t use_fixed_uq;
    /** 1=上层应复位电流 PI。 */
    uint8_t pi_reset;
    /** 1=上层应按当前电压做无扰交接。 */
    uint8_t pi_bumpless;
    /** 上一拍 Uq，单位 V。 */
    float uq_prev;
    /** 上一拍 Ud，单位 V。 */
    float ud_prev;
} motor_startup_step_t;

void motor_startup_init(motor_context_t *ctx);
void motor_startup_arm(motor_context_t *ctx);
motor_startup_step_t motor_startup_tick(motor_context_t *ctx, float theta_enc_park);
void motor_startup_finish_tick(motor_context_t *ctx, float iq_meas,
                               motor_startup_step_t *step);
m1_startup_state_t motor_startup_get_state(void);

#ifdef __cplusplus
}
#endif

#endif
