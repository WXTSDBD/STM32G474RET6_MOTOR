/**
 * @file emf_pll.h
 * @date 2026-10-06
 * @brief 反电势正交 Type-II PLL。默认不进 Park。
 *
 * 输入 Veq 或 SMO 的 eαβ。鉴相 ε 正比 |e|sin(θ−θ̂)，与 atan2(-eα, eβ) 同约定。
 * update 只允许从电流环节拍调用。
 *
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
 */

#ifndef MOTOR_OBSERVER_EMF_PLL_H
#define MOTOR_OBSERVER_EMF_PLL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    /** 内部电角，反电势原始帧，尚未扣偏置，单位 rad。 */
    float theta;
    /** 电角速度，单位 rad/s。 */
    float omega_el;
    /** PI 积分，送到 ω。 */
    float integrator;
    /** 比例增益。 */
    float kp;
    /** 积分增益。 */
    float ki;
    /** ω 绝对值上限，单位 rad/s。 */
    float omega_limit;
    /** 积分绝对值上限，单位 rad/s。 */
    float integrator_limit;
    /** 本拍鉴相 ε。 */
    float last_pd;
    /** |eαβ|。 */
    float emag;
    /** 扣偏置后、与编码器同帧的 θ̂，单位 rad。 */
    float theta_hat;
    /** wrap(θ̂ − θ_enc)，单位 rad。 */
    float theta_err;
    /** 1=已经用第一帧对齐过。 */
    uint8_t primed;
} emf_pll_t;

void emf_pll_init(emf_pll_t *p);
void emf_pll_reset(emf_pll_t *p);

void emf_pll_update(emf_pll_t *p,
                    float e_alpha, float e_beta,
                    float theta_enc,
                    float dt);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_OBSERVER_EMF_PLL_H */
