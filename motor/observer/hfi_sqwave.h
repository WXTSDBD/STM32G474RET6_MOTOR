/**
 * @file hfi_sqwave.h
 * @date 2026-10-06
 * @brief 脉振方波 HFI：估计角、注入电压、起动阶段与电流给定。
 *
 * 电流环不要直接 include 本头，走 observer_composite.h。
 * on_angle、on_current、park_theta、override_voltage 与注入读取
 * 只允许从电流环节拍调用。telem_read 给任务/VOFA。
 * consume_pi_reset、take_polarity_flip 读一次清一次，不要在任务里调。
 *
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
 */

#ifndef MOTOR_OBSERVER_HFI_SQWAVE_H
#define MOTOR_OBSERVER_HFI_SQWAVE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * 方波 HFI 起动阶段。数值与 OBS_STAGE_* 对齐，禁止改编号。
 * IDLE：未开始。MOVE：摆位。SETTLE：等待电流落稳。MEAS：踢段或测量。
 * LOG：记下结果。DONE：捕获结束且不再交接。CRAWL：低速离零。RUN：速度环。
 */
typedef enum {
    /** 未开始。 */
    HFI_STAGE_IDLE = 0,
    /** 摆位，把转子或估计角送到预定位置。 */
    HFI_STAGE_MOVE = 1,
    /** 等待电流和估计角落稳。 */
    HFI_STAGE_SETTLE = 2,
    /** 踢段或测量。 */
    HFI_STAGE_MEAS = 3,
    /** 记下本格结果，不再改 Park。 */
    HFI_STAGE_LOG = 4,
    /** 捕获结束且不再交接。 */
    HFI_STAGE_DONE = 5,
    /** 低速离零。 */
    HFI_STAGE_CRAWL = 6,
    /** 速度环运行。 */
    HFI_STAGE_RUN = 7
} hfi_stage_t;

/**
 * 捕锁状态。未锁时不要用 θ̂ 做 Park。
 */
typedef enum {
    /** 还在捕，Park 不要吃 θ̂。 */
    HFI_LOCK_CAPTURE = 0,
    /** 已锁，允许用 θ̂。 */
    HFI_LOCK_LOCKED = 1,
    /** 故障锁，停在安全态。 */
    HFI_LOCK_FAULT = 2
} hfi_lock_t;

void hfi_sqwave_init(void);
void hfi_sqwave_reset(void);

void hfi_sqwave_on_angle(float theta_enc_el, float dt);
void hfi_sqwave_on_current(float id, float iq, float i_alpha, float i_beta);

void hfi_sqwave_set_omega_ff_el(float omega_el_rad_s);

float hfi_sqwave_park_theta(float theta_enc_el);
uint8_t hfi_sqwave_override_voltage(float *ud, float *uq);
void hfi_sqwave_get_inj(float *ud_inj, float *uq_inj);
void hfi_sqwave_get_inj_ab(float *u_alpha_inj, float *u_beta_inj);
float hfi_sqwave_get_iq_ref(void);
float hfi_sqwave_get_id_ref(void);

uint8_t hfi_sqwave_speed_run_active(void);
float hfi_sqwave_get_speed_ref_rpm(void);
uint8_t hfi_sqwave_if_leave_active(void);

hfi_stage_t hfi_sqwave_get_stage(void);
hfi_lock_t hfi_sqwave_get_lock(void);
float hfi_sqwave_get_theta_cmd(void);
float hfi_sqwave_get_theta_hat(void);
float hfi_sqwave_get_theta_err(void);
float hfi_sqwave_get_eps(void);
float hfi_sqwave_get_pll_vesc_err(void);
float hfi_sqwave_get_di_q(void);
float hfi_sqwave_get_di_d(void);
float hfi_sqwave_get_x_raw(void);
float hfi_sqwave_get_y_raw(void);
float hfi_sqwave_get_x_lp(void);
float hfi_sqwave_get_y_lp(void);
uint8_t hfi_sqwave_demod_probe_freeze(void);
uint8_t hfi_sqwave_feed_coast_active(void);
float hfi_sqwave_get_vh_sign(void);
float hfi_sqwave_get_omega_el(void);
float hfi_sqwave_get_omega_trim_el(void);
float hfi_sqwave_get_pll_int_el(void);
uint8_t hfi_sqwave_take_polarity_flip(void);
void hfi_sqwave_set_inj_scale(float scale);
void hfi_sqwave_set_id_pi_release(uint8_t enable);
void hfi_sqwave_set_id_pi_soft_cmd(float scale);
float hfi_sqwave_id_pi_soft_scale(void);
float hfi_sqwave_get_id_pi_soft(void);
uint8_t hfi_sqwave_id_pi_bypass(void);
void hfi_sqwave_set_torque_theta(float theta, uint8_t enable);
void hfi_sqwave_set_hat_hold(uint8_t hold);
void hfi_sqwave_set_hat_coast_el(float omega_el);
void hfi_sqwave_seed_hat(float theta_el, float omega_el);
void hfi_sqwave_set_iq_auth_hold(uint8_t hold);
void hfi_sqwave_flip_hat_pi(void);
float hfi_sqwave_get_omega_shadow_el(void);
float hfi_sqwave_get_eps_d(void);
float hfi_sqwave_get_axis_flip_n(void);
float hfi_sqwave_get_lq_well_flip_n(void);
uint8_t hfi_sqwave_axis_ok(void);
float hfi_sqwave_get_iq_auth_abs(void);
uint8_t hfi_sqwave_iq_auth_ok(void);
uint8_t hfi_sqwave_qk_pre_ok(void);
float hfi_sqwave_get_qk_pre_flip_n(void);
float hfi_sqwave_get_eps_dead(void);
uint8_t hfi_sqwave_get_ipd_phase(void);
float hfi_sqwave_get_ipd_pulse_ud(void);

uint8_t hfi_sqwave_get_qkick_phase(void);
float hfi_sqwave_get_qkick_seed(void);
float hfi_sqwave_get_qkick_dth(void);
float hfi_sqwave_get_qkick_verdict(void);
uint8_t hfi_sqwave_consume_pi_reset(void);
uint8_t hfi_sqwave_get_sensed_cal_loop(void);

/**
 * ISR 末尾发布的 HFI 遥测快照。任务和 VOFA 只读这份，不要直抠 getter。
 * 电流环仍用原来的 getter。
 */
typedef struct {
    /** 解调误差 ε。 */
    float eps;
    /** VESC 型误差，单位 rad。 */
    float pll_vesc_err;
    /** 滤波后的 saliency x。 */
    float x_lp;
    /** 滤波后的 saliency y。 */
    float y_lp;
    /** 未滤波 saliency x。 */
    float x_raw;
    /** 未滤波 saliency y。 */
    float y_raw;
    /** 半周差分得到的 di_d。 */
    float di_d;
    /** 半周差分得到的 di_q。 */
    float di_q;
    /** PLL 积分项，单位 rad/s 电。 */
    float pll_int_el;
    /** 踢段编码器位移，单位电角 rad。 */
    float qkick_dth;
    /** 踢段判决：+1 符合，-1 不符，0 几乎无运动。 */
    float qkick_verdict;
    /** PRE 逃逸翻 +π/2 次数，浮点便于遥测。 */
    float qk_pre_flip_n;
    /** Lq 井翻 +π/2 次数，浮点便于遥测。 */
    float lq_well_flip_n;
    /** 注入符号，+1 或 -1。 */
    float vh_sign;
    /** Id→Ud 软开权重 [0,1]。 */
    float id_pi_soft;
    /** d 轴注入电压，单位 V。 */
    float ud_inj;
    /** q 轴注入电压，单位 V。 */
    float uq_inj;
    /** 有感标定圈号，非标定恒 0。 */
    uint8_t sensed_cal_loop;
    /** 1=质量过线。 */
    uint8_t iq_auth_ok;
    /** 1=准踢门禁过。 */
    uint8_t qk_pre_ok;
    /** 1=本拍冻 θ̂。 */
    uint8_t demod_probe_freeze;
    /** 1=FEED 中段滑行窗。 */
    uint8_t feed_coast_active;
} hfi_telem_snap_t;

void hfi_sqwave_telem_publish(void);
void hfi_sqwave_telem_harvest(hfi_telem_snap_t *out);
void hfi_sqwave_telem_read(hfi_telem_snap_t *out);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_OBSERVER_HFI_SQWAVE_H */
