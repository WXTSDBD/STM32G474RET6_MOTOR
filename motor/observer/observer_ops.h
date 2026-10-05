/**
 * @file observer_ops.h
 * @brief 可冻观测器 / inj 槽（两相位，禁止单次 update）。
 *
 * 冻结（2026-10-05，GATE 141 C1 CSV 21:12 过线）：本头函数指针签名、
 * OBS_STAGE_* / OBS_LOCK_* 数值、observer_view_t 字段序。禁止再改；
 * 要加源另开结构。电流环不看见 hfi_stage_t。
 */
#ifndef MOTOR_OBSERVER_OPS_H
#define MOTOR_OBSERVER_OPS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 与历史 HFI_STAGE_* 数值一致，避免 141 遥测错位。 */
#define OBS_STAGE_IDLE   0u
#define OBS_STAGE_MOVE   1u
#define OBS_STAGE_SETTLE 2u
#define OBS_STAGE_MEAS   3u
#define OBS_STAGE_LOG    4u
#define OBS_STAGE_DONE   5u
#define OBS_STAGE_CRAWL  6u
#define OBS_STAGE_RUN    7u

#define OBS_LOCK_CAPTURE 0u
#define OBS_LOCK_LOCKED  1u
#define OBS_LOCK_FAULT   2u

typedef struct {
    void (*init)(void);
    void (*reset)(void);
    void (*pre_park)(float theta_enc, float dt);
    float (*get_theta)(void);
    void (*post_park)(float id, float iq, float i_alpha, float i_beta);
    float (*get_omega)(void);
    uint8_t (*is_converged)(void);
} observer_ops_t;

typedef struct {
    uint8_t (*override_voltage)(float *ud, float *uq);
    void (*get_inj)(float *ud_inj, float *uq_inj);
    void (*get_inj_ab)(float *u_alpha_inj, float *u_beta_inj);
} observer_inj_ops_t;

/** 遥测/对照只读视图。控制路径不要靠这一份（consume/take 有副作用）。 */
typedef struct {
    uint8_t stage;
    uint8_t lock;
    float theta_cmd;
    float theta_hat;
    float theta_err;
    float eps;
    float di_q;
    float di_d;
    float x_raw;
    float y_raw;
    float vh_sign;
    float eps_dead;
    float pll_int_el;
    float omega_el;
    float omega_trim_el;
    uint8_t ipd_phase;
    float ipd_pulse_ud;
    float eps_d;
    float qkick_verdict;
    float axis_flip_n;
    uint8_t axis_ok;
    uint8_t qkick_phase;
    float qkick_seed;
    float qkick_dth;
} observer_view_t;

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_OBSERVER_OPS_H */
