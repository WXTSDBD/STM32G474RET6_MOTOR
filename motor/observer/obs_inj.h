/**
 * @file obs_inj.h
 * @date 2026-10-06
 * @brief 电流环电压相三入口：改写 ud/uq、读 dq 注入、读 αβ 注入。
 *
 * 不进 obs_angle。只允许从电流环节拍、在电压环之后调用。
 *
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
 */

#ifndef MOTOR_OBSERVER_OBS_INJ_H
#define MOTOR_OBSERVER_OBS_INJ_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

uint8_t obs_override_voltage(float *ud, float *uq);
void obs_get_inj(float *ud_inj, float *uq_inj);
void obs_get_inj_ab(float *u_alpha_inj, float *u_beta_inj);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_OBSERVER_OBS_INJ_H */
