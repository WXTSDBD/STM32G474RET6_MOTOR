/**
 * @file hfi_pub.h
 * @date 2026-10-06
 * @brief HFI 与 SMO 之间的发布角：何时切出去、何时关注入。
 *
 * 电流环走 Composite。obs_angle 只读 HFI Park，发布覆盖在 Composite get_theta。
 * hfi_pub_step 只允许从电流环节拍调用。任务只读 ss、rpm 快照。
 *
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
 */

#ifndef MOTOR_OBSERVER_HFI_PUB_H
#define MOTOR_OBSERVER_HFI_PUB_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void hfi_pub_step(float theta_smo, float omega_el, float dth);
uint8_t hfi_pub_smo_active(void);
float hfi_pub_theta(void);
float hfi_pub_ss(void);
float hfi_pub_smo_rpm(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_OBSERVER_HFI_PUB_H */
