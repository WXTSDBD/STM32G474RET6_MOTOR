/**
 * @file motor_outer_loop.h
 * @brief M1 外环调度器（2 kHz）：按 outer_mode 产生 iq_ref。
 *
 * E1 实现 DISABLED / TORQUE / SPEED / POSITION 四态。
 * motor_current_tick @ 20kHz 每 M1_SPEED_DECIM 拍调一次外环（默认 2 kHz）。
 * POSITION：P 外环再分频 M1_POS_DECIM（默认 500 Hz），速度 PI 仍 2 kHz。
 * 标定/ident 期间 outer_mode=DISABLED，保持现有行为。
 */

#ifndef MOTOR_OUTER_LOOP_H
#define MOTOR_OUTER_LOOP_H

#include "motor_context.h"
#include "motor_params_m1.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 外环初始化：速度 PI 参数 + 默认 DISABLED。
 * @note motor_current_init 末尾调用；PLL 已就绪后可调。
 */
void motor_outer_loop_init(motor_context_t *ctx);

/**
 * @brief 2 kHz 外环调度：按 outer_mode 写 ctx->id_ref / ctx->iq_ref。
 *
 * 读 PLL ω_mech（20 kHz 已更新的缓存）；DISABLED 不写。
 */
void motor_outer_loop_tick(motor_context_t *ctx);

/**
 * @brief 切换外环模式（非 ISR，含 bumpless）。
 *
 * @param ctx       控制上下文
 * @param new_mode  目标模式
 * @param iq_meas   当前 Iq 测量值 [A]（bumpless PI 用）
 * @param omega_now 当前 ω_mech [rpm]（bumpless PI 用）
 */
void motor_outer_set_mode(motor_context_t *ctx,
                           m1_outer_mode_t new_mode,
                           float iq_meas,
                           float omega_now);

/** POSITION：θ_ref ← 当前 θ_mech（上电相对零位 / 切换无跳） */
void motor_outer_arm_position_hold(motor_context_t *ctx);

/**
 * PLL/编码器就绪后同步 ω 斜坡起点与速度 PI（模式已是 SPEED 时 motor_outer_set_mode 会早退）。
 * @param omega_now 用作 bumpless 的 ω_fb，并默认作为斜坡起点
 */
void motor_outer_sync_speed_boot(motor_context_t *ctx,
                                  float iq_meas,
                                  float omega_now);

/** 仅改 ω 斜坡起点（不碰 PI）；IF 交接钉指令速时用 */
void motor_outer_set_omega_ramp_rpm(float omega_rpm);

#if M1_IF_OBS_EW_CLAMP_ENABLE
/** I/F→OBS 交接后武装 |e_ω| 限幅计时（在 speed boot 时调用） */
void motor_outer_if_obs_ew_guard_arm(void);
#endif

#if M1_IF_OBS_SOFT_BRAKE_ENABLE
/** BLEND/OBS 交接武装浅刹车：短时抬高 iq 下限，防大负流砸停 */
void motor_outer_if_obs_soft_brake_arm(motor_context_t *ctx);
#endif

#if M1_IF_OBS_CRUISE_ENABLE
/**
 * 巡航：①浅刹 →门控→ ②iq_min=-0.6 →门控→ ③iq_min=-1.2 →(可选)正常 PI。
 */
void motor_outer_if_obs_cruise_arm(motor_context_t *ctx, float omega_fb);
void motor_outer_if_obs_cruise_tick(motor_context_t *ctx, float omega_fb, float dt);
#if M1_IF_OBS_DIR_SEQ_ENABLE
/** 1=滑行段：速度环应松手，Iq/Id 强制 0 */
uint8_t motor_outer_if_obs_dir_seq_is_coast(void);
/** 进滑行时清一次观测器；返回 1 一次 */
uint8_t motor_outer_if_obs_dir_seq_consume_obs_reset(void);
/** 近零后请求再起 I/F；返回 1 一次，调用方负责 reset+arm 反转 */
uint8_t motor_outer_if_obs_dir_seq_consume_rearm(void);
#endif
#endif

#if M1_SPEED_REVERSAL_TEST_ENABLE
/** ±M1_SPEED_REVERSAL_RPM 交替；dbg.open_seq 201=FWD 202=REV */
void motor_speed_reversal_arm(motor_context_t *ctx);
#endif

#if M1_POS_STEP_TEST_ENABLE
/** θ(0) 相对阶跃；open_seq 229+step_idx，255=done（26 档） */
void motor_pos_step_test_arm(motor_context_t *ctx);
#endif

#if M1_SPEED_PROFILE_ENABLE
/** 重置阶梯 profile：首档 M1_SPEED_PROFILE_RPM_START，并写 ctx->omega_ref */
void motor_speed_profile_arm(motor_context_t *ctx);

/** repeat_en=0：跑完 100→900 后置 ladder_done；=1 循环 */
void motor_speed_profile_arm_ex(motor_context_t *ctx, uint8_t repeat_en);

/** ONE_SHOT ① 段结束消费（返回 1 一次） */
uint8_t motor_speed_profile_consume_ladder_done(void);
#endif

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_OUTER_LOOP_H */
