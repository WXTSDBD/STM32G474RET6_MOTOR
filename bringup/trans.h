#ifndef TRANS_H
#define TRANS_H
#include "tim.h"
extern float  SPEED_TARGET;
extern float  POS_TARGET;
void Park_Transform(float Ialpha, float Ibeta, float theta, float *Id, float *Iq);
void Anti_Park_Transform(float mod_d, float mod_q, float theta, float *mod_alpha, float *mod_beta);
void Clarke_Transform(float Ia, float Ib, float Ic, float *Ialpha, float *Ibeta);
float speed_cal_angle(float angle_use,float angle_use_last);
float _normalizeAngle(float angle) ;
void setPhaseVoltage(TIM_HandleTypeDef*htim, float Uq, float Ud, float angle_el) ;

#endif



