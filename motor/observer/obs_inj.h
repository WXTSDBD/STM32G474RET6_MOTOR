/**
 * @file obs_inj.h
 * @brief P7: 电流环电压相三入口（转发 override_voltage / get_inj / get_inj_ab）。
 *
 * 不进 obs_angle。给定相（id/iq/ω ref）本步不收。
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
