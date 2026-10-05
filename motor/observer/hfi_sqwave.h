/**
 * @file hfi_sqwave.h
 * @brief 脉振方波 HFI
 *
 * MOTION_BYPASS：有感速度环托底
 * IPD_SWEEP：Ud 摆位 + 多轮双脉冲消 π（不跑速度环，结果 LOG 供事后分析）
 * QKICK_SWEEP：Ud 摆位 + 故意种 θ̂=θ0/θ0+π + q 轴电流踢判极性
 * QKICK_AFTER_LOCK：锁 2θ 后单次 q 踢；START=HOLD→CAPTURE→HANDOVER→RUN
 * 或 CRAWL/Iq阶跃/短I–f / SPEED
 * DELTA_SWEEP：静态开环 θ̂=enc+δ 扫 δ，只解调不出 PLL
 * INJECT_POST_LOOP：Vh 沿 θ̂ 叠到 αβ（s_vh_sign），不进 Ud_pi
 * DEMOD_INJ_AXIS：半周差分用注入轴 iαβ，不吃 Park Id
 */
#ifndef MOTOR_OBSERVER_HFI_SQWAVE_H
#define MOTOR_OBSERVER_HFI_SQWAVE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    HFI_STAGE_IDLE = 0,
    HFI_STAGE_MOVE = 1,
    HFI_STAGE_SETTLE = 2,
    HFI_STAGE_MEAS = 3,
    HFI_STAGE_LOG = 4,
    HFI_STAGE_DONE = 5,
    HFI_STAGE_CRAWL = 6,
    HFI_STAGE_RUN = 7
} hfi_stage_t;

/** 捕锁状态：未锁不用 θ̂ 做 Park */
typedef enum {
    HFI_LOCK_CAPTURE = 0,
    HFI_LOCK_LOCKED = 1,
    HFI_LOCK_FAULT = 2
} hfi_lock_t;

void hfi_sqwave_init(void);
void hfi_sqwave_reset(void);

void hfi_sqwave_on_angle(float theta_enc_el, float dt);
void hfi_sqwave_on_current(float id, float iq, float i_alpha, float i_beta);

/**
 * 喂入电角速度前馈 [rad/s]。
 * Step2：CAPTURE 侧由上层填 ω_ref；LOCKED 可仍填 enc（见 OMEGA_FF_FROM_REF）。
 */
void hfi_sqwave_set_omega_ff_el(float omega_el_rad_s);

float hfi_sqwave_park_theta(float theta_enc_el);
uint8_t hfi_sqwave_override_voltage(float *ud, float *uq);
void hfi_sqwave_get_inj(float *ud_inj, float *uq_inj);
void hfi_sqwave_get_inj_ab(float *u_alpha_inj, float *u_beta_inj);
float hfi_sqwave_get_iq_ref(void);
float hfi_sqwave_get_id_ref(void);

/** 1=应开有感速度环（RUN 段）；δ 扫描恒为 0 */
uint8_t hfi_sqwave_speed_run_active(void);
/** RUN 内速度指令序列 [rpm]；Step4 CRAWL I–f 时为 ω_cmd */
float hfi_sqwave_get_speed_ref_rpm(void);
/** 1=CRAWL 内短 I–f 离零进行中（Park=θ_if） */
uint8_t hfi_sqwave_if_leave_active(void);

hfi_stage_t hfi_sqwave_get_stage(void);
hfi_lock_t hfi_sqwave_get_lock(void);
/** 旁路：速度指令；δ 扫描：当前 δ [rad] */
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
/** 1=本拍冻 θ̂（解调探查窗）；未开探查恒为 0 */
uint8_t hfi_sqwave_demod_probe_freeze(void);
/** 1=FEED 中段滑行窗（Iq*=0）；未开 COAST 恒为 0 */
uint8_t hfi_sqwave_feed_coast_active(void);
float hfi_sqwave_get_vh_sign(void);
float hfi_sqwave_get_omega_el(void);
/** HFI PLL 修正量 δω=Kp·eps+∫（不含 enc 前馈）[rad/s 电] */
float hfi_sqwave_get_omega_trim_el(void);
/** HFI PLL 积分项 ∫ [rad/s 电]；VOFA 常乘 rpm_scale 看假速 */
float hfi_sqwave_get_pll_int_el(void);
/**
 * 速度环开头极性反了时翻过一次 π。读一次后清掉。
 * 未开这路恒为 0。
 */
uint8_t hfi_sqwave_take_polarity_flip(void);
/** 注入幅值乘上 scale，1 为标称 Vh，0 为关掉。交接用。 */
void hfi_sqwave_set_inj_scale(float scale);
/**
 * 1：允许恢复 Id PI（交接 Park 已到 SMO 且注入已灭）。
 * 上升沿启动 Ud 软开；未放行时即使 scale=0 仍旁路。
 */
void hfi_sqwave_set_id_pi_release(uint8_t enable);
/**
 * 交接外给定 Id→Ud 权重 [0,1]。scale<0 回到自动爬坡。
 * OVERLAP 路径每拍写；abort/idle 清掉。
 */
void hfi_sqwave_set_id_pi_soft_cmd(float scale);
/**
 * 放行后 Id→Ud 权重 [0,1]。有外给定用外给定；否则自动爬坡。
 */
float hfi_sqwave_id_pi_soft_scale(void);
/** 只读当前 soft（不推进自动斜坡计数）；遥测用 */
float hfi_sqwave_get_id_pi_soft(void);
/**
 * 1=旁路 Id PI（Ud_pi 置 0，只留注入）；q 环仍开。
 * 踢后 RUN：注入开着或尚未 id_pi_release 时为 1；放行后为 0（走软开）。
 * 未开 M1_HFI_ID_PI_OFF_ENABLE 恒 0。
 */
uint8_t hfi_sqwave_id_pi_bypass(void);
/**
 * α>0 时力矩角不是 θ̂。enable=1 后注入和解调按这个角旋回 θ̂。
 * enable=0 恢复「Park 就是 θ̂」。
 */
void hfi_sqwave_set_torque_theta(float theta, uint8_t enable);
/** 1：ε 不再写入 θ̂。角度改按保持速度每拍加 ω·Ts。 */
void hfi_sqwave_set_hat_hold(uint8_t hold);
/** 保持期间改用电角速度 [rad/s]。注入收完、开始交角度时跟上 SMO。 */
void hfi_sqwave_set_hat_coast_el(float omega_el);
/**
 * @brief SMO→HFI 反向交接：用观测角/电角速度重播 θ̂ 与 PLL 积分，并解除 hat_hold。
 * @param theta_el 电角度 [rad]
 * @param omega_el 电角速度 [rad/s]
 */
void hfi_sqwave_seed_hat(float theta_el, float omega_el);
/** 1：质量门不再因为 x 掉下去把 |Iq| 天花板收到 0。 */
void hfi_sqwave_set_iq_auth_hold(uint8_t hold);
/** 只翻 HFI 的 θ̂ 并清其积分。不通知 SMO。 */
void hfi_sqwave_flip_hat_pi(void);
/** 旁路 PLL 转速 [rad/s 电]：跟主环 θ̂、无 enc 前馈，不进 Park */
float hfi_sqwave_get_omega_shadow_el(void);
/** 正交 di_d 解调（选轴）；未开 AXIS_SEL 为 0 */
float hfi_sqwave_get_eps_d(void);
/** 选轴翻 ±90° 累计次数（浮点便于 VOFA） */
float hfi_sqwave_get_axis_flip_n(void);
/** Lq 井一次 +π/2；未开为 0 */
float hfi_sqwave_get_lq_well_flip_n(void);
/** 1=已确认真 d（可放开 Iq）；未开 AXIS_SEL 恒为 1 */
uint8_t hfi_sqwave_axis_ok(void);
/** 无感质量给出的 |Iq| 天花板 [A]；未开 IQ_AUTH 为全速环上限 */
float hfi_sqwave_get_iq_auth_abs(void);
/** 1=质量过线（|x| 近 A 持续）；未开 IQ_AUTH 恒为 1 */
uint8_t hfi_sqwave_iq_auth_ok(void);
/** 1=PRE 假锁门禁过（准踢）；未开 PRE_GATE 恒为 1 */
uint8_t hfi_sqwave_qk_pre_ok(void);
/** PRE 逃逸翻 +π/2 次数（VOFA） */
float hfi_sqwave_get_qk_pre_flip_n(void);
/** 当前 eps 死区（扫档时变化） */
float hfi_sqwave_get_eps_dead(void);
/** IPD 子相位：0 ALIGN / 1 SETTLE0 / 2 P0 / 3 SETTLE1 / 4 P1；扫位 MOVE 时为 0 */
uint8_t hfi_sqwave_get_ipd_phase(void);
/** 本格脉冲电压指令 [V]；非 IPD 为 0 */
float hfi_sqwave_get_ipd_pulse_ud(void);

/** QKICK 子相位：0 SEED / 1 KICK / 2 BRAKE；非 MEAS 为 0 */
uint8_t hfi_sqwave_get_qkick_phase(void);
/** 本格种子：0=θ̂=θ_cmd，1=θ̂=θ_cmd+π */
float hfi_sqwave_get_qkick_seed(void);
/** MCU 测得踢段 Δθ_enc [rad]，LOG 段保持 */
float hfi_sqwave_get_qkick_dth(void);
/** MCU 判决：+1 符号符合种子期望，-1 不符，0 几乎无运动 */
float hfi_sqwave_get_qkick_verdict(void);
/** 进入 KICK 时置 1，上层读一次后清零 → 复位电流 PI */
uint8_t hfi_sqwave_consume_pi_reset(void);
/** 有感标定圈号（0-based）；非标定模式恒 0 */
uint8_t hfi_sqwave_get_sensed_cal_loop(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_OBSERVER_HFI_SQWAVE_H */
