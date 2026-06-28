/**
 * @file dbg_monitor.h
 * @brief VOFA / Watch 调试镜像（与 CubeMX main 解耦，Service 层可安全 include）。
 *
 * 实例 `dbg` 定义在 debug/dbg_monitor.c；App 层通过 main.h 间接包含本头亦可。
 */

#ifndef DBG_MONITOR_H
#define DBG_MONITOR_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t csr;
    uint8_t en;
    uint8_t intout;
    uint16_t pggain;
    uint16_t adc_jdr1;
    uint32_t adc_irq_cnt;
    uint32_t hal_state;
} DbgOpampChan_t;

typedef struct {
    DbgOpampChan_t opamp[3];
    int16_t adc_shunt[3];
    int16_t adc_reg[3];
    int32_t adc_offset[3];
    int16_t adc_zeroed[3];
    float adc_ia;
    float adc_ib;
    float adc_ic;
    float foc_ia;
    float foc_ib;
    float foc_ic;
    float foc_theta_el;
    float foc_id;
    float foc_iq;
    float foc_id_ref;
    float foc_ud_pi;
    float foc_uq_pi;
    float foc_uq_out;
    uint8_t open_seq_phase;
    uint8_t startup_state;
    float startup_omega_mech_rpm;
    float enc_cal_add;
    float enc_cal_add_raw;
    uint8_t phase_cal_ok;
    uint8_t binding_loaded;
    uint8_t phase_cal_fail;
    uint8_t phase_cal_fail_reason;
    uint8_t phase_cal_channels_done;
    float phase_cal_last_snr;
    uint8_t pwm_ch_to_phase_dbg[3];
    uint8_t adc_rank_to_phase_dbg[3];
    int8_t phase_sign_dbg[3];
    uint8_t phase_cal_pwm_idx;
    uint8_t phase_cal_delta_idx;
    uint8_t phase_cal_st;
    float phase_cal_bipolar_lsb[3][3][3];
    float phase_cal_dom_s_lsb[3];
    float id_acdc;
    float iq_acdc;
    uint8_t deadband_cal_len;
    uint8_t deadband_cal_outlier;
    uint8_t deadband_cal_state;
    uint8_t deadband_nvm_loaded;
    uint8_t deadband_mode;
} DbgMon_t;

extern volatile DbgMon_t dbg;

#ifdef __cplusplus
}
#endif

#endif /* DBG_MONITOR_H */
