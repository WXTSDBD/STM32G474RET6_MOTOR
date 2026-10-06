/**
 * @file hfi_pub.h
 * @brief GATE 141 发布角。电流环走 Composite；obs_angle 只读 getter。
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
