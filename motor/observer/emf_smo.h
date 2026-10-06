/**
 * @file emf_smo.h
 * @date 2026-10-06
 * @brief 电流滑模观测器。默认不进 Park。系数在 init 里预计算。
 *
 * update 只允许从电流环节拍调用。LPF 调度按机械转速改截止频率。
 *
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
 */

#ifndef MOTOR_OBSERVER_EMF_SMO_H
#define MOTOR_OBSERVER_EMF_SMO_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    /** 估计 α 电流，单位 A。 */
    float ihat_alpha;
    /** 估计 β 电流，单位 A。 */
    float ihat_beta;
    /** 1=已经吃过第一帧。 */
    uint8_t primed;
    /** 本拍 α 电压，单位 V。 */
    float u_alpha;
    /** 本拍 β 电压，单位 V。 */
    float u_beta;
    /** 估计 eα，单位 V。 */
    float e_alpha;
    /** 估计 eβ，单位 V。 */
    float e_beta;
    /** |eαβ|。 */
    float emag;
    /** 由反电势取出的电角，单位 rad。 */
    float theta_hat;
    /** wrap(θ̂ − θ_enc)，单位 rad。 */
    float theta_err;
    /** 电角速度，单位 rad/s。本结构里可能未每拍更新。 */
    float omega_el;
} emf_smo_t;

void emf_smo_init(emf_smo_t *o);
void emf_smo_reset(emf_smo_t *o);

void emf_smo_update(emf_smo_t *o,
                    float i_alpha, float i_beta,
                    float u_alpha, float u_beta,
                    float theta_enc,
                    float omega_mech_rpm);

uint8_t emf_smo_lpf_sched_update(float omega_mech_rpm);
float emf_smo_get_lpf_hz(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_OBSERVER_EMF_SMO_H */
