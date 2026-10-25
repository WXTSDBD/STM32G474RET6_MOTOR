/**
 * @file deadband_id_cal.c
 * @date 2026-10-06
 * @brief Id 锁轴扫表编排实现。

 *
 * 节拍限制见 deadband_id_cal.h 文件头。
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
 */

#include "deadband_id_cal.h"

#include "deadband_cal.h"
#include "deadband_service.h"
#include "foc_pi.h"
#include "dbg_monitor.h"
#include "motor_cfg.h"
#include "motor_params_m1.h"
#include "rs_ident.h"
#include "ld_lq_ident.h"
#include "telem_ident_dump.h"
#include "telem_lut_dump.h"

void deadband_id_cal_init(void) {}
void deadband_id_cal_tick(motor_context_t *ctx) { (void)ctx; }
uint8_t deadband_id_cal_is_running(void) { return 0u; }
uint8_t deadband_id_cal_is_done(void) { return 0u; }
/* use_fix_theta / target_theta / align_ud*: stubs in deadband_id_cal.h */
uint8_t deadband_id_cal_in_iq_probe(void) { return 0u; }
uint8_t deadband_id_cal_use_cal_pi_limits(void) { return 0u; }
float deadband_id_cal_clamp_id_ref(float ref) { return ref; }
float deadband_id_cal_clamp_iq_ref(float ref) { return ref; }
uint8_t deadband_id_cal_bumpless_arm(void) { return 0u; }
float deadband_id_cal_bumpless_id_ref(void) { return 0.0f; }
void deadband_id_cal_consume_bumpless_arm(void) {}
uint8_t deadband_id_cal_iq_bumpless_arm(void) { return 0u; }
float deadband_id_cal_iq_bumpless_ref(void) { return 0.0f; }
void deadband_id_cal_consume_iq_bumpless_arm(void) {}
uint8_t deadband_id_cal_should_capture(void) { return 0u; }
float deadband_id_cal_capture_id_ref(void) { return 0.0f; }
/* capture_append_dlut: stub in deadband_id_cal.h when !DUAL_ANGLE */
void deadband_id_cal_clear_capture_pending(void) {}
void deadband_id_cal_sync_dbg(void) {}
