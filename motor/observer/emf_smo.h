/**
 * @file emf_smo.h
 * @brief 有感旁路：经典电流滑模 SMO（不进 Park）；系数 init 预计算。
 */
#ifndef MOTOR_OBSERVER_EMF_SMO_H
#define MOTOR_OBSERVER_EMF_SMO_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float ihat_alpha;
    float ihat_beta;
    uint8_t primed;
    float u_alpha;
    float u_beta;
    float e_alpha;
    float e_beta;
    float emag;
    float theta_hat;
    float theta_err;
    float omega_el;
} emf_smo_t;

void emf_smo_init(emf_smo_t *o);
void emf_smo_reset(emf_smo_t *o);

void emf_smo_update(emf_smo_t *o,
                    float i_alpha, float i_beta,
                    float u_alpha, float u_beta,
                    float theta_enc,
                    float omega_mech_rpm);

/**
 * 按机械转速调度 e-LPF：
 * - LINEAR：fc=clip(k·fe,min,max)，fe=|n|·p/60（推荐）
 * - 否则分档表（若 M1_EMF_SMO_LPF_SCHED_ENABLE）
 * @return 分档时为档号 0..N-1；线性时返回 0。
 */
uint8_t emf_smo_lpf_sched_update(float omega_mech_rpm);

/** 当前 LPF 截止频率 [Hz]（调度或固定宏）。 */
float emf_smo_get_lpf_hz(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_OBSERVER_EMF_SMO_H */
