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
    /** 编码器电角（rad），不受 FIX_THETA 覆盖；L 辨识 Δθ 监测用 */
    float enc_theta_el;
    float foc_id;
    float foc_iq;
    float foc_id_ref;
    float foc_iq_ref;
    float foc_ud_pi;
    float foc_uq_pi;
    float foc_uq_out;
    float foc_ud_out;
    float foc_svpwm_uref;
    float foc_svpwm_duty_dev;
    float foc_svpwm_duty_ab;
    float foc_vd_est;
    float foc_vq_est;
    float foc_pwm_ccr1;
    float foc_pwm_ccr2;
    float foc_pwm_ccr3;
    /** SVPWM+deadband 后逻辑 duty（0..1），写 CCR 前 */
    float foc_duty_ta;
    float foc_duty_tb;
    float foc_duty_tc;
    uint8_t foc_svpwm_sector;
    uint8_t open_seq_phase;
    uint8_t startup_state;
    float startup_omega_mech_rpm;
    /** PLL @ 20 kHz：机械转速与相位误差（Park 未切 PLL 时可与 startup_omega 对比） */
    float pll_omega_mech_rpm;
    float pll_theta_err_rad;
    float pll_omega_diff_rpm;
    /** PLL ω 减编码器裸差分 ω [rpm]，VOFA ch11 */
    float pll_omega_err_rpm;
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
    /** Rs ramp 辨识（Pass0+Rs）；open_seq 54/55/56 */
    float rs_ident_ohm;
    float rs_ident_intercept_v;
    float rs_ident_n;
    uint8_t rs_ident_ok;
    uint8_t rs_ident_round;
    /** Rs/L 双轮辨识：0=OFF 轮，1=LUT 轮（VOFA ch10） */
    float rs_l_ident_lut_round;
    /** VASI 20 点 Ld/Lq（Pass0+Rs 后）；open_seq 57/58/59 */
    float ld_lq_grid_idx;
    float ld_lq_id_bias;
    float ld_lq_iq_bias;
    float ld_lq_L_est_uH;
    float ld_lq_v_inj;
    float ld_lq_rs_used;
    float ld_lq_n_ld_ok;
    float ld_lq_n_lq_ok;
    uint8_t ld_lq_axis;
    uint8_t ld_lq_sub;
    uint8_t ld_lq_ok;
    float ld_lq_theta_drift_mech_deg;
    float ld_lq_f_hz;
    uint8_t ld_lq_angle_leg;
    float ld_lq_theta_target_el;
    /** VASI 过程量（Watch / VOFA ch8–11 镜像）；open_seq 57 时有效 */
    float ld_lq_proc_grid;
    float ld_lq_proc_amp_idx;
    float ld_lq_proc_coarse_l_uH;
    float ld_lq_proc_last_l_uH;
    float ld_lq_proc_psi_du_wb;
    float ld_lq_proc_di_den_a;
    float ld_lq_proc_half_ticks;
    /** sub*100 + axis*10 + coarse*5 + amp_idx（解码见 tools/parse_ident_vofa.py） */
    float ld_lq_proc_code;
    /** 外环：模式 / 速度 / 位置 / iq_ref */
    uint8_t outer_mode;
    float outer_omega_ref;
    float outer_omega_mech_rpm;
    float outer_theta_ref_rad;
    float outer_theta_mech_rad;
    float outer_theta_err_rad;
    float outer_iq_ref;
    /** 速度阶梯 profile 当前档 0..4（100/300/500/700/900 rpm） */
    uint8_t outer_profile_step;
} DbgMon_t;

extern volatile DbgMon_t dbg;

#ifdef __cplusplus
}
#endif

#endif /* DBG_MONITOR_H */
