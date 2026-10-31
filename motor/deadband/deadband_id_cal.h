/**
 * @file deadband_id_cal.h
 * @date 2026-10-06
 * @brief Rs → l_hfi 编排状态机（建表 Pass0 已下线）。
 *
 * tick 只允许从电流环节拍、经 deadband_flow 调用。
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
 *       API 始终声明；关 M1_ID_LOCK_CAL_SWEEP 时 .c 提供空桩。
 */

#ifndef DEADBAND_ID_CAL_H
#define DEADBAND_ID_CAL_H

#include "motor_context.h"
#include "motor_params_m1.h"

#ifdef __cplusplus
extern "C" {
#endif

void deadband_id_cal_init(void);
void deadband_id_cal_tick(motor_context_t *ctx);

typedef void (*deadband_id_cal_pass0_cont_fn)(motor_context_t *ctx);
void deadband_id_cal_set_pass0_continue(deadband_id_cal_pass0_cont_fn fn);
void deadband_id_cal_continue_rs_lhfi(motor_context_t *ctx);
void deadband_id_cal_boot_rs_l_hfi_only(motor_context_t *ctx);

uint8_t deadband_id_cal_is_running(void);
uint8_t deadband_id_cal_is_done(void);

uint8_t deadband_id_cal_use_fix_theta(void);
float deadband_id_cal_target_theta(void);
uint8_t deadband_id_cal_use_align_ud(void);
float deadband_id_cal_align_ud_v(void);
uint8_t deadband_id_cal_use_post_ident_hold_ud(void);
uint8_t deadband_id_cal_in_rs_ident(void);
uint8_t deadband_id_cal_in_l_hfi_ident(void);
uint8_t deadband_id_cal_in_l_hfi_pre_decay(void);
uint8_t deadband_id_cal_use_l_hfi_align_ud(void);
static inline uint8_t deadband_id_cal_in_l_hfi_sweep(void)
{
    return deadband_id_cal_in_l_hfi_ident();
}

uint8_t deadband_id_cal_in_iq_probe(void);
uint8_t deadband_id_cal_use_cal_pi_limits(void);
float deadband_id_cal_clamp_id_ref(float ref);
float deadband_id_cal_clamp_iq_ref(float ref);

uint8_t deadband_id_cal_bumpless_arm(void);
float deadband_id_cal_bumpless_id_ref(void);
void deadband_id_cal_consume_bumpless_arm(void);
uint8_t deadband_id_cal_iq_bumpless_arm(void);
float deadband_id_cal_iq_bumpless_ref(void);
void deadband_id_cal_consume_iq_bumpless_arm(void);

uint8_t deadband_id_cal_should_capture(void);
float deadband_id_cal_capture_id_ref(void);
static inline uint8_t deadband_id_cal_capture_append_dlut(void) { return 1u; }
void deadband_id_cal_clear_capture_pending(void);
void deadband_id_cal_sync_dbg(void);

#ifdef __cplusplus
}
#endif

#endif /* DEADBAND_ID_CAL_H */
