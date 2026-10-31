/**
 * @file isr_state.c
 * @date 2026-10-29
 * @brief ISR 共享状态定义（字段常驻，无能力 #if）。
 */
#include "isr/isr_state.h"

motor_context_t g_isr_m1_ctx;

emf_smo_t g_isr_emf_smo;
emf_pll_t g_isr_emf_pll;

float g_isr_speed_fb_rpm;
motor_pll_t g_isr_m1_pll;
uint16_t g_isr_pll_prime_ticks;
float g_isr_pll_omega_mech_rpm;
float g_isr_theta_mech_pll_rad;

float g_isr_theta_mech_rad;

uint8_t g_isr_speed_slow_div;
volatile uint8_t g_isr_outer_run_enable = 1u;
volatile uint32_t g_isr_diag_tim6_cnt;
volatile uint32_t g_isr_diag_foc_cnt;

float g_isr_hfi_recon_theta;
uint8_t g_isr_hfi_recon_theta_ok;
const observer_inj_ops_t *g_isr_m1_inj;
