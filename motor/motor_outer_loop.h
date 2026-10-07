/**
 * @file motor_outer_loop.h
 * @date 2026-10-06
 * @brief M1 外环：按模式写出 id_ref / iq_ref。
 *
 * motor_outer_loop_tick 由电流环节拍按 M1_SPEED_DECIM 分频调用，仍在 ADC 注入完成中断里。
 * 不要在普通任务里再调 tick。set_mode、arm、profile 给任务用。
 * 位置环再按 M1_POS_DECIM 分频；速度 PI 仍跟 tick 同频。
 * 标定期间 outer_mode 保持 DISABLED。
 *
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
 */

#ifndef MOTOR_OUTER_LOOP_H
#define MOTOR_OUTER_LOOP_H

#include "motor_context.h"
#include "motor_params_m1.h"

#ifdef __cplusplus
extern "C" {
#endif

void motor_outer_loop_init(motor_context_t *ctx);
void motor_outer_loop_tick(motor_context_t *ctx);
void motor_outer_set_mode(motor_context_t *ctx,
                           m1_outer_mode_t new_mode,
                           float iq_meas,
                           float omega_now);
void motor_outer_arm_position_hold(motor_context_t *ctx);
void motor_outer_sync_speed_boot(motor_context_t *ctx,
                                  float iq_meas,
                                  float omega_now);
void motor_outer_set_omega_ramp_rpm(float omega_rpm);

#if M1_IF_OBS_EW_CLAMP_ENABLE
void motor_outer_if_obs_ew_guard_arm(void);
#endif

#if M1_IF_OBS_SOFT_BRAKE_ENABLE
void motor_outer_if_obs_soft_brake_arm(motor_context_t *ctx);
#endif

#if M1_IF_OBS_CRUISE_ENABLE
void motor_outer_if_obs_cruise_arm(motor_context_t *ctx, float omega_fb);
void motor_outer_if_obs_cruise_tick(motor_context_t *ctx, float omega_fb, float dt);
#if M1_IF_OBS_DIR_SEQ_ENABLE
uint8_t motor_outer_if_obs_dir_seq_is_coast(void);
uint8_t motor_outer_if_obs_dir_seq_consume_obs_reset(void);
uint8_t motor_outer_if_obs_dir_seq_consume_rearm(void);
#endif
#endif

#if M1_SPEED_REVERSAL_TEST_ENABLE
void motor_speed_reversal_arm(motor_context_t *ctx);
#endif

#if M1_POS_STEP_TEST_ENABLE
void motor_pos_step_test_arm(motor_context_t *ctx);
#endif

#if M1_OUTER_NEST_ENABLE && \
    ((M1_OUTER_EXPT == M1_OUTER_EXPT_POS_STEP) || \
     (M1_OUTER_EXPT == M1_OUTER_EXPT_POS_REV) || \
     (M1_OUTER_EXPT == M1_OUTER_EXPT_MIT_HOLD) || \
     (M1_OUTER_EXPT == M1_OUTER_EXPT_MIT_REV))
void motor_outer_pos_mini_arm(motor_context_t *ctx);
#endif

#if M1_OUTER_NEST_ENABLE && (M1_OUTER_EXPT == M1_OUTER_EXPT_SIGNOFF)
void motor_outer_signoff_arm(motor_context_t *ctx);
uint8_t motor_outer_signoff_is_armed(void);
void motor_outer_signoff_tick(motor_context_t *ctx,
                              float theta_fb_rad,
                              float omega_rpm);
#endif

#if M1_SPEED_PROFILE_ENABLE
void motor_speed_profile_arm(motor_context_t *ctx);
void motor_speed_profile_arm_ex(motor_context_t *ctx, uint8_t repeat_en);
uint8_t motor_speed_profile_consume_ladder_done(void);
#endif

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_OUTER_LOOP_H */
