/**
 * @file hfi_spd_obs.h
 * @date 2026-10-06
 * @brief SMO 机械转速滑动平均。不是 obs_cfg 配置项。
 *
 * 由 Composite 在电流环节拍里步进。窗口长度改了会改变发布门槛的平滑程度。
 *
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
 */

#ifndef MOTOR_OBSERVER_HFI_SPD_OBS_H
#define MOTOR_OBSERVER_HFI_SPD_OBS_H

/**
 * 滑动平均窗口拍数。默认 400，约 20 ms @20 kHz。
 * 加长更稳、更慢；缩短更跟手、噪声更大。
 */
#ifndef HFI_SMO_W_MA_N
#define HFI_SMO_W_MA_N 400u
#endif

void hfi_smo_w_ma_reset(void);
float hfi_smo_w_ma_step(float rpm);

#endif /* MOTOR_OBSERVER_HFI_SPD_OBS_H */
