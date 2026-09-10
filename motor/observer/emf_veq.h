/**
 * @file emf_veq.h
 * @brief 有感旁路：电压方程反电势 + atan（不进 Park）
 *
 * 热路径只吃已算好的 uαβ；系数在 init 预计算（同电流环 Kp/Ki 思路）。
 */
#ifndef MOTOR_OBSERVER_EMF_VEQ_H
#define MOTOR_OBSERVER_EMF_VEQ_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float i_alpha_prev;
    float i_beta_prev;
    uint8_t primed;
    float u_alpha;
    float u_beta;
    float i_alpha;
    float i_beta;
    float e_alpha;
    float e_beta;
    float emag;
    float theta_hat;
    float theta_err;
    float omega_el;
    float psi_inst;
} emf_veq_t;

void emf_veq_init(emf_veq_t *o);
void emf_veq_reset(emf_veq_t *o);

/** @param u_alpha/u_beta  已由 ud/uq + θ_park 反 Park（与 FOC 同 sin/cos） */
void emf_veq_update(emf_veq_t *o,
                    float i_alpha, float i_beta,
                    float u_alpha, float u_beta,
                    float theta_enc,
                    float omega_mech_rpm);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_OBSERVER_EMF_VEQ_H */
