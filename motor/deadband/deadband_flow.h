/**
 * @file deadband_flow.h
 * @date 2026-10-06
 * @brief 辨识配方调度：Rs→l_hfi / ident / speed_ident。

 *
 * tick 只允许从电流环节拍调用。boot 给上电一次。
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
 */

#ifndef DEADBAND_FLOW_H
#define DEADBAND_FLOW_H

#include "motor_context.h"
#include "motor_params_m1.h"

#ifdef __cplusplus
extern "C" {
#endif

void deadband_flow_boot(motor_context_t *ctx);
void deadband_flow_tick(motor_context_t *ctx);
uint8_t deadband_flow_id_cal_active(void);
uint8_t deadband_flow_consume_ident_pi_retune(void);

#if M1_IDENT_ENABLE
uint8_t deadband_flow_in_ident(void);
#else
static inline uint8_t deadband_flow_in_ident(void) { return 0u; }
#endif

static inline uint8_t deadband_flow_speed_ladder_active(void) { return 0u; }

#if M1_SPEED_IDENT_ENABLE
uint8_t deadband_flow_speed_ident_active(void);
#else
static inline uint8_t deadband_flow_speed_ident_active(void) { return 0u; }
#endif

#ifdef __cplusplus
}
#endif

#endif /* DEADBAND_FLOW_H */
