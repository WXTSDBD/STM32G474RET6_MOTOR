/**
 * @file observer_ops.h
 * @date 2026-10-06
 * @brief 可冻观测器与注入槽。两相位，禁止单次 update 连打两拍。
 *
 * 本头冻住：函数指针签名、OBS_STAGE_* / OBS_LOCK_* 数值、observer_view_t 字段序。
 * 电流环看见的是这些编号，不要再暴露 hfi_stage_t。
 *
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
 */

#ifndef MOTOR_OBSERVER_OPS_H
#define MOTOR_OBSERVER_OPS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * 起动阶段编号，与 hfi_stage_t 数值一致。改编号会让遥测错位。
 */
#define OBS_STAGE_IDLE   0u
#define OBS_STAGE_MOVE   1u
#define OBS_STAGE_SETTLE 2u
#define OBS_STAGE_MEAS   3u
#define OBS_STAGE_LOG    4u
#define OBS_STAGE_DONE   5u
#define OBS_STAGE_CRAWL  6u
#define OBS_STAGE_RUN    7u

/** 捕锁编号，与 hfi_lock_t 数值一致。 */
#define OBS_LOCK_CAPTURE 0u
#define OBS_LOCK_LOCKED  1u
#define OBS_LOCK_FAULT   2u

/**
 * 观测器热路径。pre_park 在 Park 前，post_park 在电流采样后。
 * 不要把两拍合成一次 update。
 */
typedef struct {
    /** 上电装配。 */
    void (*init)(void);
    /** 清内部状态。 */
    void (*reset)(void);
    /** Park 前：吃电角和节拍，推进估计。 */
    void (*pre_park)(float theta_enc, float dt);
    /** 本拍 Park 用的电角，单位 rad。 */
    float (*get_theta)(void);
    /** Park 后：吃 Id/Iq 与 αβ 电流，做解调。 */
    void (*post_park)(float id, float iq, float i_alpha, float i_beta);
    /** 电角速度。现行 HFI 给出 PLL 积分项。 */
    float (*get_omega)(void);
    /** 1=已锁，允许当估计角用。 */
    uint8_t (*is_converged)(void);
} observer_ops_t;

/**
 * 注入相。override 在电压环之后改 ud/uq；get_inj 给 dq，get_inj_ab 给 αβ。
 */
typedef struct {
    /** 1=本拍改写了 ud/uq。指针不可为 NULL。 */
    uint8_t (*override_voltage)(float *ud, float *uq);
    /** 读 dq 注入，单位 V。 */
    void (*get_inj)(float *ud_inj, float *uq_inj);
    /** 读 αβ 注入，单位 V。 */
    void (*get_inj_ab)(float *u_alpha_inj, float *u_beta_inj);
} observer_inj_ops_t;

/**
 * 遥测只读视图。控制路径不要靠这一份；consume/take 有副作用，不在这里。
 */
typedef struct {
    /** 起动阶段，取值 OBS_STAGE_*。 */
    uint8_t stage;
    /** 捕锁，取值 OBS_LOCK_*。 */
    uint8_t lock;
    /** 指令电角，单位 rad。 */
    float theta_cmd;
    /** 估计电角 θ̂，单位 rad。 */
    float theta_hat;
    /** wrap(θ̂ − θ_enc)，单位 rad。 */
    float theta_err;
    /** 解调误差 ε。 */
    float eps;
    /** 半周差分 di_q。 */
    float di_q;
    /** 半周差分 di_d。 */
    float di_d;
    /** 未滤波 saliency x。 */
    float x_raw;
    /** 未滤波 saliency y。 */
    float y_raw;
    /** 注入符号。 */
    float vh_sign;
    /** 当前 ε 死区，单位 rad。 */
    float eps_dead;
    /** PLL 积分项，单位 rad/s 电。 */
    float pll_int_el;
    /** 电角速度，单位 rad/s。 */
    float omega_el;
    /** PLL 修正量，单位 rad/s 电。 */
    float omega_trim_el;
    /** IPD 子相位。 */
    uint8_t ipd_phase;
    /** IPD 脉冲电压，单位 V。 */
    float ipd_pulse_ud;
    /** 正交解调 ε_d。 */
    float eps_d;
    /** 踢段判决。 */
    float qkick_verdict;
    /** 选轴翻次数，浮点便于遥测。 */
    float axis_flip_n;
    /** 1=已确认真 d。 */
    uint8_t axis_ok;
    /** 踢段子相位。 */
    uint8_t qkick_phase;
    /** 本格种子：0=θ_cmd，1=θ_cmd+π。 */
    float qkick_seed;
    /** 踢段位移，单位电角 rad。 */
    float qkick_dth;
} observer_view_t;

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_OBSERVER_OPS_H */
