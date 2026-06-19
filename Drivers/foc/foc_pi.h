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

float foc_pi_step(foc_pi_t *pi, float ref, float fb);

#ifdef __cplusplus
}
#endif

#endif
