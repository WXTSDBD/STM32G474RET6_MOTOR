/**
 * @file obs_angle.h
 * @date 2026-10-06
 * @brief 电流环角路径三入口：Park 前、取角、Park 后。
 *
 * 转发到 HFI 的 on_angle / park_theta / on_current。发布覆盖不在本头。
 * 三个函数只允许从电流环节拍调用。
 *
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
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
