/**
 * @file deadband_id_cal.h
 * @date 2026-10-06
 * @brief Id 锁轴扫表状态机：Pass0、commit、Iq 探路。

 *
 * tick 只允许从电流环节拍调用。
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
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

#if M1_ID_CAL_IQ_PROBE_ONLY_ENABLE
/** 跳过 Pass0/align，直接进入 Iq 探路 OFF 段。 */
void deadband_id_cal_boot_iq_probe_only(motor_context_t *ctx);
#endif

#if M1_ID_CAL_RS_LD_LQ_ONLY_ENABLE
/** 跳过 Pass0：NVM 恢复 LUT + OFF；ALIGN→HOLD→VASI（Rs 固定 M1_RS_OHM）由 init 状态机接管。 */
void deadband_id_cal_boot_rs_ld_lq_only(motor_context_t *ctx);
#endif

#if M1_LD_LQ_IDENT_ENABLE && M1_DEADBAND_FLOW_LD_LQ_AFTER_OPEN_UD
/** Pass0+Ud 阶梯后：保留 RAM LUT，ALIGN→HOLD→VASI（不 deadband_cal_reset）。 */
void deadband_id_cal_reinit_ld_lq_chain(motor_context_t *ctx);
#endif

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
#if M1_VOFA_IDENT_DUMP_ENABLE
uint8_t deadband_id_cal_use_post_ident_hold_ud(void);
#else
static inline uint8_t deadband_id_cal_use_post_ident_hold_ud(void) { return 0u; }
#endif
#else
static inline uint8_t deadband_id_cal_use_align_ud(void) { return 0u; }
static inline float deadband_id_cal_align_ud_v(void) { return 0.0f; }
static inline uint8_t deadband_id_cal_use_post_ident_hold_ud(void) { return 0u; }
#endif

uint8_t deadband_id_cal_in_iq_probe(void);
#if M1_RS_IDENT_ENABLE
uint8_t deadband_id_cal_in_rs_ident(void);
#else
static inline uint8_t deadband_id_cal_in_rs_ident(void) { return 0u; }
#endif
#if M1_LD_LQ_IDENT_ENABLE
uint8_t deadband_id_cal_in_ld_lq_ident(void);
uint8_t deadband_id_cal_in_ld_lq_pre_decay(void);
uint8_t deadband_id_cal_use_ld_lq_align_ud(void);
#if M1_LD_LQ_MULTI_ANGLE_ENABLE
uint8_t deadband_id_cal_in_ld_lq_sweep(void);
#else
static inline uint8_t deadband_id_cal_in_ld_lq_sweep(void)
{
    return deadband_id_cal_in_ld_lq_ident();
}
#endif
#else
static inline uint8_t deadband_id_cal_in_ld_lq_ident(void) { return 0u; }
static inline uint8_t deadband_id_cal_in_ld_lq_pre_decay(void) { return 0u; }
static inline uint8_t deadband_id_cal_in_ld_lq_sweep(void) { return 0u; }
static inline uint8_t deadband_id_cal_use_ld_lq_align_ud(void) { return 0u; }
#endif
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
