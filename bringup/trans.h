#ifndef TRANS_H
#define TRANS_H
#include "tim.h"
extern float  SPEED_TARGET;
extern float  POS_TARGET;

void Park_Transform(float Ialpha, float Ibeta, float theta, float *Id, float *Iq);

/** 热路径 Park：sin/cos 由调用方 motor_trig_sincos(theta) 一次算出，避免 arm_sin/cos。 */
void Park_Transform_sc(float Ialpha, float Ibeta,
                       float sin_el, float cos_el,
                       float *Id, float *Iq);

void Anti_Park_Transform(float mod_d, float mod_q, float theta, float *mod_alpha, float *mod_beta);

/** 热路径反 Park：与 Park_Transform_sc 共用同一组 sin_el/cos_el。 */
void Anti_Park_Transform_sc(float mod_d, float mod_q,
                            float sin_el, float cos_el,
                            float *mod_alpha, float *mod_beta);

void Clarke_Transform(float Ia, float Ib, float Ic, float *Ialpha, float *Ibeta);
float speed_cal_angle(float angle_use,float angle_use_last);
float _normalizeAngle(float angle) ;
void setPhaseVoltage(TIM_HandleTypeDef*htim, float Uq, float Ud, float angle_el) ;

#endif



