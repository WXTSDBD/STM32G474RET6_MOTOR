/**
 * @file obs_theta_notch.h
 * @brief 转速自适应陷波：Park 前压 θ̂ 的机械 1/rev（可选 2/rev）纹波
 *
 * 对 wrap(θ̂−θ_lp) 做 notch，避免对绕卷角度直接滤波。
 * 仅在 OBS 使能路径调用；算力约一个二阶 biquad/拍。
 */
#ifndef MOTOR_OBSERVER_OBS_THETA_NOTCH_H
#define MOTOR_OBSERVER_OBS_THETA_NOTCH_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void obs_theta_notch_init(void);
void obs_theta_notch_reset(void);

/**
 * @param theta_hat   PLL 电角 [rad]
 * @param omega_obs_rpm 观测机械转速 [rpm]（定 f0=|n|·rpm/60）
 * @param enable      1=处于 OBS 等允许陷波的状态
 * @param dt          控制周期 [s]
 * @return 陷波后电角 [rad]（未使能则原样返回）
 */
float obs_theta_notch_apply(float theta_hat,
                            float omega_obs_rpm,
                            uint8_t enable,
                            float dt);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_OBSERVER_OBS_THETA_NOTCH_H */
