/**
 * @file ident_module.h
 * @brief 堵转辨识激励发生器：Iq 阶跃 / Bode sin 扫频（不含 deadband 切换）。
 *
 * VOFA Bode：ch3=Iq_fb ch4=Iq_ref ch5=Uq_pi
 * open_seq 由 ident_flow 维护：60=HOLD 62/63/64=Bode 73=DONE
 */

#ifndef IDENT_MODULE_H
#define IDENT_MODULE_H

#include "motor_context.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    IDENT_MOD_HOLD = 0,
    IDENT_MOD_STEP,
    IDENT_MOD_BODE,
    IDENT_MOD_DONE,
} ident_module_state_t;

void ident_module_init(motor_context_t *ctx);
void ident_module_tick(motor_context_t *ctx);

ident_module_state_t ident_module_get_state(void);
uint8_t ident_module_get_round(void);
uint8_t ident_module_get_phase_in_round(void);

float ident_module_iq_ref_cmd(void);
float ident_module_bode_freq_hz(void);

/** STEP 段轮次；非 STEP 返回 0 */
uint8_t ident_module_step_round(void);

/** BODE 段轮次 0..ROUNDS-1；非 BODE 返回 0 */
uint8_t ident_module_bode_round(void);

/** @deprecated 使用 ident_flow_deadband_fixed() */
uint8_t ident_module_deadband_fixed(void);

#ifdef __cplusplus
}
#endif

#endif /* IDENT_MODULE_H */
