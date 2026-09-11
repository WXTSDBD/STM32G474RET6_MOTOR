/**
 * @file obs_soft_switch.h
 * @brief 编码器 θ → EMF-PLL θ̂ 软切（融合 + 故障回退）
 *
 * OBS 态下 Park 用 θ̂；若打开速度同步切，则速度反馈同步用 θ̂→motor_pll。
 */
#ifndef MOTOR_OBSERVER_OBS_SOFT_SWITCH_H
#define MOTOR_OBSERVER_OBS_SOFT_SWITCH_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    OBS_SS_ENC = 0,       /* 纯编码器 */
    OBS_SS_ARM = 1,       /* 门限累计中 */
    OBS_SS_BLEND = 2,     /* 融合中 */
    OBS_SS_OBS = 3,       /* 纯观测角（+可选观测速） */
    OBS_SS_FALLBACK = 4   /* 已回退，本跑锁定编码器 */
} obs_ss_state_t;

void obs_soft_switch_init(void);
void obs_soft_switch_reset(void);

/**
 * @param omega_enc_rpm  编码器机械转速 [rpm]（武装进入门限）
 * @param omega_obs_rpm  观测机械转速 [rpm]（OBS/BLEND 掉速回退用，避免编码器毛刺误踢）
 */
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

/** 1=已切到观测角（OBS），可供速度环同步切反馈 */
uint8_t obs_soft_switch_speed_on_obs(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_OBSERVER_OBS_SOFT_SWITCH_H */
