/**
 * @file isr_state.h
 * @date 2026-10-29
 * @brief 20 kHz ISR 共享状态（原 motor_current.c 文件内 static）。
 *
 * init/getter 仍在 motor_current.c；热路径编排在 isr_entry.c。
 * 对象常驻；调用方用 if (M1_*) / 空指针，不再用字段级 #if。
 */
#ifndef MOTOR_ISR_ISR_STATE_H
#define MOTOR_ISR_ISR_STATE_H

#include "motor_context.h"
#include "motor_params_m1.h"
#include "motor_pll.h"
#include "observer/emf_pll.h"
#include "observer/emf_smo.h"
#include "observer/observer_composite.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

extern motor_context_t g_isr_m1_ctx;

extern emf_smo_t g_isr_emf_smo;
extern emf_pll_t g_isr_emf_pll;

extern float g_isr_speed_fb_rpm;
extern motor_pll_t g_isr_m1_pll;
extern uint16_t g_isr_pll_prime_ticks;
extern float g_isr_pll_omega_mech_rpm;
extern float g_isr_theta_mech_pll_rad;

extern float g_isr_theta_mech_rad;

extern uint8_t g_isr_speed_slow_div;
extern volatile uint8_t g_isr_outer_run_enable;
extern volatile uint32_t g_isr_diag_tim6_cnt;
extern volatile uint32_t g_isr_diag_foc_cnt;

extern float g_isr_hfi_recon_theta;
extern uint8_t g_isr_hfi_recon_theta_ok;
extern const observer_inj_ops_t *g_isr_m1_inj;

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_ISR_ISR_STATE_H */
