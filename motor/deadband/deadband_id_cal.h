/**
 * @file deadband_id_cal.h
 * @brief Id 锁轴扫表编排（Pass0 / commit / Iq 探路）。
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

/** 状态机未进入 DONE */
uint8_t deadband_id_cal_is_running(void);
uint8_t deadband_id_cal_is_done(void);

#if M1_ID_CAL_FIX_THETA_ENABLE
uint8_t deadband_id_cal_use_fix_theta(void);
float deadband_id_cal_target_theta(void);
#else
static inline uint8_t deadband_id_cal_use_fix_theta(void) { return 0u; }
static inline float deadband_id_cal_target_theta(void) { return 0.0f; }
#endif

#if M1_ID_CAL_ALIGN_ENABLE && M1_ID_CAL_FIX_THETA_ENABLE
uint8_t deadband_id_cal_use_align_ud(void);
float deadband_id_cal_align_ud_v(void);
#else
static inline uint8_t deadband_id_cal_use_align_ud(void) { return 0u; }
static inline float deadband_id_cal_align_ud_v(void) { return 0.0f; }
#endif

uint8_t deadband_id_cal_in_iq_probe(void);
uint8_t deadband_id_cal_use_cal_pi_limits(void);
float deadband_id_cal_clamp_id_ref(float ref);

uint8_t deadband_id_cal_bumpless_arm(void);
float deadband_id_cal_bumpless_id_ref(void);
void deadband_id_cal_consume_bumpless_arm(void);
uint8_t deadband_id_cal_iq_bumpless_arm(void);
float deadband_id_cal_iq_bumpless_ref(void);
void deadband_id_cal_consume_iq_bumpless_arm(void);

uint8_t deadband_id_cal_should_capture(void);
float deadband_id_cal_capture_id_ref(void);
#if M1_ID_CAL_DUAL_ANGLE_ENABLE
uint8_t deadband_id_cal_capture_append_dlut(void);
#else
static inline uint8_t deadband_id_cal_capture_append_dlut(void) { return 1u; }
#endif
void deadband_id_cal_clear_capture_pending(void);

void deadband_id_cal_sync_dbg(void);

#ifdef __cplusplus
}
#endif

#endif /* DEADBAND_ID_CAL_H */
