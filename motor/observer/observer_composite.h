/**
 * @file observer_composite.h
 * @brief HFI Composite：两相位 ops + 独立 inj 槽。电流环不 include hfi_sqwave.h。
 *
 * 冻结（同 observer_ops.h）：init / ops() / inj_ops() / bind_emf_pll /
 * 下列控制 getter（含 consume_pi_reset、take_polarity_flip 副作用）不得改签名。
 * 禁止收成一次 update。141 pub overlay 在 Composite get_theta，angle 壳只做 Park。
 */
#ifndef MOTOR_OBSERVER_COMPOSITE_H
#define MOTOR_OBSERVER_COMPOSITE_H

#include <stdint.h>
#include "observer/observer_ops.h"

#ifdef __cplusplus
extern "C" {
#endif

void observer_composite_init(void);
const observer_ops_t *observer_ops(void);
/** 无注入时返回 NULL。GATE 141 冻结为 HFI inj 表。 */
const observer_inj_ops_t *observer_inj_ops(void);
void observer_bind_emf_pll(void *pll);

void observer_telem_publish(void);
void observer_pub_step(float theta_smo, float omega_el, float dth);
float observer_pub_ss(void);
float observer_pub_smo_rpm(void);
void observer_smo_w_ma_reset(void);
float observer_smo_w_ma_step(float rpm);
void observer_read_view(observer_view_t *v);

uint8_t observer_get_stage(void);
uint8_t observer_get_lock(void);
float observer_get_pll_int_el(void);
float observer_get_omega_el(void);
uint8_t observer_consume_pi_reset(void);
uint8_t observer_speed_run_active(void);
float observer_get_speed_ref_rpm(void);
float observer_get_id_ref(void);
float observer_get_iq_ref(void);
uint8_t observer_if_leave_active(void);
void observer_set_omega_ff_el(float omega_el_rad_s);
float observer_get_iq_auth_abs(void);
uint8_t observer_id_pi_bypass(void);
float observer_id_pi_soft_scale(void);
float observer_get_theta_hat(void);
uint8_t observer_take_polarity_flip(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_OBSERVER_COMPOSITE_H */
