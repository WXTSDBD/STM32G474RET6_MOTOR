/**
 * @file observer_composite.h
 * @date 2026-10-06
 * @brief 无感对外聚合口：ops、注入、发布角、EMF-PLL 包装、低速槽。
 *
 * 电流环只通过本头拿观测器，不要 include hfi_sqwave.h。
 * 热路径函数随电流环节拍调用。consume_pi_reset、take_polarity_flip
 * 有读清副作用，禁止在任务里调。
 * ops 与 inj 函数指针签名已冻，不要改；要加源另开结构。
 *
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
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
const observer_inj_ops_t *observer_inj_ops(void);
void observer_bind_emf_pll(void *pll);
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
