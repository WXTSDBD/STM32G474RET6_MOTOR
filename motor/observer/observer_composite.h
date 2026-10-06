/**
 * @file observer_composite.h
 * @brief HFI Composite 聚合口。电流环不 include hfi_sqwave.h。
 *
 * 冻结（同 observer_ops.h，不得改签名）：init / ops() / inj_ops() / bind_emf_pll /
 * 控制 getter（含 consume_pi_reset、take_polarity_flip 副作用）。禁止收成一次 update。
 * 141 pub overlay 在 Composite get_theta，angle 壳只做 Park。
 *
 * 收口新增（可改实现）：observer_bringup、observer_emf_*、observer_lo_*、
 * observer_pub_*、observer_smo_w_ma_*、observer_read_view、
 * observer_telem_publish。低速槽类型在 obs_src.h。
 */
#ifndef MOTOR_OBSERVER_COMPOSITE_H
#define MOTOR_OBSERVER_COMPOSITE_H

#include <stdint.h>
#include "observer/observer_ops.h"
#include "observer/obs_src.h"

#ifdef __cplusplus
extern "C" {
#endif

void observer_composite_init(void);
const observer_ops_t *observer_ops(void);
/** 无注入时返回 NULL。GATE 141 冻结为 HFI inj 表。 */
const observer_inj_ops_t *observer_inj_ops(void);
void observer_bind_emf_pll(void *pll);
/** init + bind，电流环 HFI 装配口。不改 ops 签名。 */
void observer_bringup(void *emf_pll);

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
/** 中高速槽（已 bind 的 EMF-PLL）只读；与 HFI θ̂ 不是同一个量。 */
float observer_emf_theta_hat(void);
float observer_emf_omega_el(void);
float observer_emf_theta_err(void);
float observer_emf_last_pd(void);
void observer_emf_reset(void);
void observer_emf_update(float e_alpha, float e_beta, float theta_enc, float dt);

void observer_lo_set_inj_scale(float scale);
void observer_lo_set_iq_auth_hold(uint8_t hold);
void observer_lo_seed_hat(float theta_el, float omega_el);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_OBSERVER_COMPOSITE_H */
