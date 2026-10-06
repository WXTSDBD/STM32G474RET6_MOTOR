/**
 * @file foc_pi.h
 * @date 2026-10-06
 * @brief 离散 PI，带回算抗饱和。输出是电压或电流指令，单位由调用方定。
 *
 * foc_pi_step 只允许从电流环或外环节拍调用。不要在任务里对同一实例重入。
 *
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
 */

#ifndef FOC_PI_H
#define FOC_PI_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    /** 比例增益。 */
    float kp;
    /** 积分增益，已经含节拍。 */
    float ki;
    /** 积分状态。 */
    float integrator;
    /** 输出上限。 */
    float out_max;
    /** 输出下限。 */
    float out_min;
    /** 积分上限。 */
    float int_max;
    /** 积分下限。 */
    float int_min;
} foc_pi_t;

void foc_pi_init(foc_pi_t *pi, float kp, float ki,
                 float out_min, float out_max,
                 float int_min, float int_max);
void foc_pi_reset(foc_pi_t *pi);
void foc_pi_bumpless(foc_pi_t *pi, float u_prev, float ref, float fb);
float foc_pi_step(foc_pi_t *pi, float ref, float fb);
float foc_pi_step_beta(foc_pi_t *pi, float ref, float fb, float beta);
void foc_pi_bumpless_beta(foc_pi_t *pi, float u_prev, float ref, float fb, float beta);
float foc_pi_step_beta_hold_i(foc_pi_t *pi, float ref, float fb, float beta,
                              uint8_t hold_i);

#ifdef __cplusplus
}
#endif

#endif
