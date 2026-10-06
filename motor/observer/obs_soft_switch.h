/**
 * @file obs_soft_switch.h
 * @date 2026-10-06
 * @brief 编码器或 I-f 角到观测角的软切。
 *
 * 角：ENC → ARM → BLEND → OBS。故障可回退并锁在编码器。
 * apply 只允许从电流环节拍调用。速度是否切观测由 speed_use_obs 另判。
 *
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
 */

#ifndef MOTOR_OBSERVER_OBS_SOFT_SWITCH_H
#define MOTOR_OBSERVER_OBS_SOFT_SWITCH_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * 软切状态。ENC 吃底角；ARM 在累计进门；BLEND 在融角；
 * OBS 角已纯观测；FALLBACK 本跑锁死编码器。
 */
typedef enum {
    /** 纯编码器或 I-f 底角。 */
    OBS_SS_ENC = 0,
    /** 门限累计中，Park 仍吃底角。 */
    OBS_SS_ARM = 1,
    /** 角融合中。 */
    OBS_SS_BLEND = 2,
    /** 纯观测角。速度反馈不一定已经切观测。 */
    OBS_SS_OBS = 3,
    /** 已回退，本跑锁定编码器。 */
    OBS_SS_FALLBACK = 4
} obs_ss_state_t;

void obs_soft_switch_init(void);
void obs_soft_switch_reset(void);

float obs_soft_switch_apply(float theta_enc,
                            float theta_hat,
                            float theta_err,
                            float emag,
                            float omega_enc_rpm,
                            float omega_obs_rpm,
                            float omega_ref,
                            float iq,
                            float dt);

obs_ss_state_t obs_soft_switch_get_state(void);
float obs_soft_switch_get_alpha(void);
uint8_t obs_soft_switch_speed_on_obs(void);
uint8_t obs_soft_switch_speed_use_obs(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_OBSERVER_OBS_SOFT_SWITCH_H */
