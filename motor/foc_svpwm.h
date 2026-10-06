/**
 * @file foc_svpwm.h
 * @date 2026-10-06
 * @brief Clarke、Park、反 Park、SVPWM 和死区占空比补偿。
 *
 * foc_svpwm_apply 只允许从电流环节拍调用。变换函数可以在同一拍里先算电流。
 *
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
 */

#ifndef FOC_SVPWM_H
#define FOC_SVPWM_H

#include "bsp_axes.h"

#ifdef __cplusplus
extern "C" {
#endif

void Park_Transform(float Ialpha, float Ibeta, float theta, float *Id, float *Iq);
void Park_Transform_sc(float Ialpha, float Ibeta,
                       float sin_el, float cos_el,
                       float *Id, float *Iq);
void Anti_Park_Transform(float mod_d, float mod_q, float theta, float *mod_alpha, float *mod_beta);
void Anti_Park_Transform_sc(float mod_d, float mod_q,
                            float sin_el, float cos_el,
                            float *mod_alpha, float *mod_beta);
void Clarke_Transform(float Ia, float Ib, float Ic, float *Ialpha, float *Ibeta);
float _normalizeAngle(float angle);
void foc_svpwm_apply(bsp_axis_t *axis, float Uq, float Ud, float angle_el);
void foc_svpwm_apply_abc(bsp_axis_t *axis,
                         float Uq, float Ud, float angle_el,
                         float ia, float ib, float ic,
                         float id_dq, float iq_dq);
int svpwm_sector_from_uq_ud(float Uq, float Ud, float angle_el);

#ifdef __cplusplus
}
#endif

#endif
