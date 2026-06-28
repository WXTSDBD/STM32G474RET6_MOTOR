/**
 * @file deadband_cal.h
 * @brief Id 锁轴扫表在线建 deadband LUT（段 2 capture / 段 3 commit）。
 * @note 模块目录：motor/deadband/
 */

#ifndef DEADBAND_CAL_H
#define DEADBAND_CAL_H

#include <stdbool.h>
#include <stdint.h>

#include "factory_nvm.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    DEADBAND_CAL_IDLE  = 0,
    DEADBAND_CAL_SWEEP = 1,
    DEADBAND_CAL_DONE  = 2,
} deadband_cal_state_t;

void deadband_cal_reset(void);

/**
 * @brief dwell 末采一点：lut_amps=|id|，lut_vals=|ud_pi - id*Rs|
 * @return true 已写入表
 */
bool deadband_cal_capture(float id_a, float ud_pi_v, float id_ref_a);

/**
 * @brief capture + geo abc 样本；append_dlut=0 时只入样本池（Pass0-B）。
 */
bool deadband_cal_capture_at(float id_a, float ud_pi_v, float id_ref_a,
                             float theta_el, uint8_t append_dlut);

/** 段 3：sort [+ dedupe] → flatten → geo plut → switch_runtime_lut(0) Pass1/Iq abc */
void deadband_cal_commit(void);

/** 切换 runtime LUT：1=d 表+Ud 路径，0=phase 表+abc duty */
void deadband_cal_switch_runtime_lut(uint8_t use_d_table);

/** 扫表状态机进入 DONE（段 2 不调 commit 时调用） */
void deadband_cal_finish_sweep(void);

deadband_cal_state_t deadband_cal_get_state(void);
uint8_t deadband_cal_len(void);
bool deadband_cal_outlier_seen(void);

/** geo abc 样本池长度（双角 Pass0） */
uint16_t deadband_cal_geo_sample_len(void);

/** Pass0 capture / --compare-lut d 轴域 */
const float *deadband_cal_amps(void);
const float *deadband_cal_vals(void);

/** commit 后 runtime abc LUT（geo 或 × cos30°）；VOFA 突发 phase A */
const float *deadband_cal_amps_phase(void);
const float *deadband_cal_vals_phase(void);
const float *deadband_cal_amps_phase_ph(uint8_t phase);
const float *deadband_cal_vals_phase_ph(uint8_t phase);
uint8_t deadband_cal_phase_lut_triplet(void);

/** commit 后导出 plut + scale 供 factory_nvm_write_deadband */
bool deadband_cal_export_nvm(factory_nvm_deadband_t *out);

/** 上电 NORMAL：从 NVM 加载 plut 并 LUT ON */
bool deadband_cal_apply_nvm(const factory_nvm_deadband_t *in);

#ifdef __cplusplus
}
#endif

#endif
