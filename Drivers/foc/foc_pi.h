/**
 * @file foc_pi.h
 * @brief 离散 PI + back-calculation 抗饱和；输出为 dq 电压（V）。
 */

#ifndef FOC_PI_H
#define FOC_PI_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float kp;
    float ki;
    float integrator;
    float out_max;
    float out_min;
    float int_max;
    float int_min;
} foc_pi_t;

void foc_pi_init(foc_pi_t *pi, float kp, float ki,
                 float out_min, float out_max,
                 float int_min, float int_max);

void foc_pi_reset(foc_pi_t *pi);

/** 无扰动预加载：使 u_prev ≈ kp*(ref-fb) + integrator 在下一步成立 */
void foc_pi_bumpless(foc_pi_t *pi, float u_prev, float ref, float fb);

float foc_pi_step(foc_pi_t *pi, float ref, float fb);

/** 2-DOF：积分用 (ref−fb)，比例用 (β·ref−fb)；β=1 等同 foc_pi_step */
float foc_pi_step_beta(foc_pi_t *pi, float ref, float fb, float beta);

void foc_pi_bumpless_beta(foc_pi_t *pi, float u_prev, float ref, float fb, float beta);

/** hold_i=1：冻结积分（输出仍可非零）；用于 SPEED_IDENT PLL settle */
float foc_pi_step_beta_hold_i(foc_pi_t *pi, float ref, float fb, float beta,
                              uint8_t hold_i);

#ifdef __cplusplus
}
#endif

#endif
