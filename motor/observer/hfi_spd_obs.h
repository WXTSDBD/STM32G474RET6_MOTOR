/**
 * @file hfi_spd_obs.h
 * @brief SMO 转速滑动平均（hfi_spd_obs.c）。不是 obs_cfg 配置项。
 */
#ifndef MOTOR_OBSERVER_HFI_SPD_OBS_H
#define MOTOR_OBSERVER_HFI_SPD_OBS_H

#ifndef HFI_SMO_W_MA_N
#define HFI_SMO_W_MA_N 400u
#endif

void hfi_smo_w_ma_reset(void);
float hfi_smo_w_ma_step(float rpm);

#endif /* MOTOR_OBSERVER_HFI_SPD_OBS_H */
