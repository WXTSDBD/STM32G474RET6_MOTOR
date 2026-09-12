/**
 * @file obs_soft_switch.h
 * @brief 编码器/I/F θ → EMF-PLL θ̂ 软切（融合 + 可选故障回退）
 *
 * 角：ENC → ARM → BLEND → OBS（Park=θ̂）。
 * 速：可选推迟（SPD_DEFER）——有感时进 OBS 后仍可先吃编码器，门限满足再切观测速。
 *     ENC_OPTIONAL=1 时角进 BLEND/OBS 即用 ω̂（不可再等延后，否则吃死编码器 PLL）。
 */
#ifndef MOTOR_OBSERVER_OBS_SOFT_SWITCH_H
#define MOTOR_OBSERVER_OBS_SOFT_SWITCH_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    OBS_SS_ENC = 0,       /* 纯编码器 / I/F 底角 */
    OBS_SS_ARM = 1,       /* 门限累计中 */
    OBS_SS_BLEND = 2,     /* 角融合中 */
    OBS_SS_OBS = 3,       /* 纯观测角（速可能仍有感） */
    OBS_SS_FALLBACK = 4   /* 已回退，本跑锁定编码器 */
} obs_ss_state_t;

void obs_soft_switch_init(void);
void obs_soft_switch_reset(void);

/**
 * @param omega_enc_rpm  编码器机械转速 [rpm]（有感进门 / 可选切速对照）
 * @param omega_obs_rpm  观测机械转速 [rpm]（I/F 进门、掉速、切速门限）
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

/** 1=角已纯 OBS（供 I/F 释放 / 速度环接管；不代表速度反馈已切观测） */
uint8_t obs_soft_switch_speed_on_obs(void);

/**
 * 1=速度反馈应走观测侧。
 * SPD_DEFER=0：BLEND|OBS（与角同步）。
 * SPD_DEFER=1：仅切速门限通过后（先角后速）。
 */
uint8_t obs_soft_switch_speed_use_obs(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_OBSERVER_OBS_SOFT_SWITCH_H */
