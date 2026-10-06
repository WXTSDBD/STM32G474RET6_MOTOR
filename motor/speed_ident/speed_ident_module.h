/**
 * @file speed_ident_module.h
 * @date 2026-10-06
 * @brief 速度环辨识激励：ω_ref 阶跃和正弦。

 *
 * tick 随外环调用。
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
 */

#ifndef SPEED_IDENT_MODULE_H
#define SPEED_IDENT_MODULE_H

#include "motor_context.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    SPEED_IDENT_HOLD = 0,
    SPEED_IDENT_STEP,
    SPEED_IDENT_BODE,
    SPEED_IDENT_DONE,
} speed_ident_module_state_t;

void speed_ident_module_init(motor_context_t *ctx);
void speed_ident_module_tick(motor_context_t *ctx);

speed_ident_module_state_t speed_ident_module_get_state(void);
uint8_t speed_ident_module_get_round(void);
uint8_t speed_ident_module_get_phase_in_round(void);

float speed_ident_module_omega_ref_cmd(void);
float speed_ident_module_bode_freq_hz(void);

uint8_t speed_ident_module_step_round(void);
uint8_t speed_ident_module_bode_round(void);

/** HOLD 前 PLL settle：1=禁止速度 PI 出 Iq */
uint8_t speed_ident_module_hold_iq_inhibit(void);

#ifdef __cplusplus
}
#endif

#endif /* SPEED_IDENT_MODULE_H */
