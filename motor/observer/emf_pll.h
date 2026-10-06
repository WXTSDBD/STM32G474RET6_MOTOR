/**
 * @file emf_pll.h
 * @brief 有感旁路：EMF 正交 Type-II PLL 取角（不进 Park）
 *
 * 输入 Veq/SMO 的 eαβ；鉴相 ε∝|e|sin(θ−θ̂)，与 atan2(-eα,eβ) 同约定。
 */
#ifndef MOTOR_OBSERVER_EMF_PLL_H
#define MOTOR_OBSERVER_EMF_PLL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float theta;       /* 内部电角（EMF 原始帧，未扣 θ_off） */
    float omega_el;    /* 电角速度 [rad/s] */
    float integrator;  /* PI 积分 → ω */
    float kp;
    float ki;
    float omega_limit;
    float integrator_limit;
    float last_pd;     /* 鉴相 ε */
    float emag;
    float theta_hat;   /* 扣偏置后，与编码器同帧 */
    float theta_err;   /* wrap(θ̂ − θ_enc) */
    uint8_t primed;
} emf_pll_t;

void emf_pll_init(emf_pll_t *p);
void emf_pll_reset(emf_pll_t *p);

/** @param e_alpha/e_beta  反电势（建议来自 Veq）；@param theta_enc 电角 */
void emf_pll_update(emf_pll_t *p,
                    float e_alpha, float e_beta,
                    float theta_enc,
                    float dt);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_OBSERVER_EMF_PLL_H */
