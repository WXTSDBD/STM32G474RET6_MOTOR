/**
 * @file emf_veq.h
 * @date 2026-10-06
 * @brief 电压方程反电势加 atan。默认不进 Park。
 *
 * 热路径只吃已经反 Park 好的 uαβ。系数在 init 里预计算。
 * update 只允许从电流环节拍调用。
 *
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
 */

#ifndef MOTOR_OBSERVER_EMF_VEQ_H
#define MOTOR_OBSERVER_EMF_VEQ_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    /** 上一拍 iα，单位 A。 */
    float i_alpha_prev;
    /** 上一拍 iβ，单位 A。 */
    float i_beta_prev;
    /** 1=已经有上一拍电流。 */
    uint8_t primed;
    /** 本拍 uα，单位 V。 */
    float u_alpha;
    /** 本拍 uβ，单位 V。 */
    float u_beta;
    /** 本拍 iα，单位 A。 */
    float i_alpha;
    /** 本拍 iβ，单位 A。 */
    float i_beta;
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
    /** 由转速换来的电角速度，单位 rad/s。 */
    float omega_el;
    /** 瞬时磁链幅值，单位 Wb。 */
    float psi_inst;
} emf_veq_t;

void emf_veq_init(emf_veq_t *o);
void emf_veq_reset(emf_veq_t *o);

void emf_veq_update(emf_veq_t *o,
                    float i_alpha, float i_beta,
                    float u_alpha, float u_beta,
                    float theta_enc,
                    float omega_mech_rpm);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_OBSERVER_EMF_VEQ_H */
