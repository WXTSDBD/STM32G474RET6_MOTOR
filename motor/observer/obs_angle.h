/**
 * @file obs_angle.h
 * @brief P6: 电流环角路径三入口（转发 HFI on_angle / park_theta / on_current）。
 *
 * theta_enc 只是今日相位 1 输入，不等于规划 §4.1。本头不是 F5。
 */
#ifndef MOTOR_OBSERVER_OBS_ANGLE_H
#define MOTOR_OBSERVER_OBS_ANGLE_H

#ifdef __cplusplus
extern "C" {
#endif

void obs_pre_park(float theta_enc, float dt);
float obs_get_theta(void);
void obs_post_park(float id, float iq, float i_alpha, float i_beta);
void obs_angle_bind_pll(void *pll);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_OBSERVER_OBS_ANGLE_H */
