/**
 * @file ident_module.h
 * @date 2026-10-06
 * @brief 堵转辨识激励：Iq 阶跃和正弦扫频。不含死区切换。

 *
 * tick 只允许从电流环节拍调用。
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
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
float ident_module_id_ref_cmd(void);
float ident_module_bode_freq_hz(void);

/** BODE 当前频点下标 0..N-1；非 BODE 返回 0 */
uint16_t ident_module_bode_freq_idx(void);

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
