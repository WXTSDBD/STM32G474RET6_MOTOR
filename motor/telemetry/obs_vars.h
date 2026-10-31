/**
 * @file obs_vars.h
 * @date 2026-10-31
 * @brief 观测镜像刷新（ISR/遥测拍末调用）。
 */
#ifndef MOTOR_TELEMETRY_OBS_VARS_H
#define MOTOR_TELEMETRY_OBS_VARS_H

#ifdef __cplusplus
extern "C" {
#endif

void obs_vars_tick_mirrors(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_TELEMETRY_OBS_VARS_H */
