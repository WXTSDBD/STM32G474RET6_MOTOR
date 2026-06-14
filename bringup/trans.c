#include "trans.h"
#include "motor_trig.h"
#include <math.h>
#include "gpio.h"
#include "tim.h"
#include "bsp_dwt.h"
#include "arm_math.h"
#include "main.h"

#define voltage_power_supply 24
#define PWM_Period 3999
#define _PI 3.14159265359f
#define _PI_2 1.57079632679f
#define _PI_3 1.0471975512f
#define _2PI 6.28318530718f
#define _3PI_2 4.71238898038f
#define _PI_6 0.52359877559f
#define _SQRT3 1.73205080757f
#define INV_SQRT3 (1.0f / _SQRT3)
#define INV_VBUS   (1.0f / (float)voltage_power_supply)
#define INV_PI3    (1.0f / _PI_3)
#define HALF_F     0.5f
#define SIN_PI3    0.86602540378f

#ifndef SVPWM_ALLOW_OVERMOD
#define SVPWM_ALLOW_OVERMOD 0
#endif

float t_s, t_last;

static void svpwm_t1_t2_from_theta(float theta, float Uref, float *T1, float *T2)
{
    float s;
    float c;
    float sin_pi3_m_theta;
    float scale;

    motor_trig_sincos(theta, &c, &s);
    sin_pi3_m_theta = SIN_PI3 * c - HALF_F * s;
    scale = _SQRT3 * Uref;
    *T1 = scale * sin_pi3_m_theta;
    *T2 = scale * s;
}

void Park_Transform(float Ialpha, float Ibeta, float theta, float *Id, float *Iq)
{
    float sin_val = arm_sin_f32(theta);
    float cos_val = arm_cos_f32(theta);

    *Id = Ialpha * cos_val + Ibeta * sin_val;
    *Iq = -Ialpha * sin_val + Ibeta * cos_val;
}

/**
 * @brief Park（热路径）：调用方已用 motor_trig_sincos 算好 sin_el/cos_el。
 *        SVPWM 扇区角不同，setPhaseVoltage 内仍单独 CORDIC，与此处无关。
 */
void Park_Transform_sc(float Ialpha, float Ibeta,
                       float sin_el, float cos_el,
                       float *Id, float *Iq)
{
    *Id = Ialpha * cos_el + Ibeta * sin_el;
    *Iq = -Ialpha * sin_el + Ibeta * cos_el;
}

void Anti_Park_Transform(float mod_d, float mod_q, float theta, float *mod_alpha, float *mod_beta)
{
    float sin_val = arm_sin_f32(theta);
    float cos_val = arm_cos_f32(theta);

    *mod_alpha = mod_d * cos_val - mod_q * sin_val;
    *mod_beta = mod_d * sin_val + mod_q * cos_val;
}

/**
 * @brief 反 Park（热路径）：与 Park_Transform_sc 共用 sin_el/cos_el（PI 阶段用）。
 */
void Anti_Park_Transform_sc(float mod_d, float mod_q,
                            float sin_el, float cos_el,
                            float *mod_alpha, float *mod_beta)
{
    *mod_alpha = mod_d * cos_el - mod_q * sin_el;
    *mod_beta = mod_d * sin_el + mod_q * cos_el;
}

void Clarke_Transform(float Ia, float Ib, float Ic, float *Ialpha, float *Ibeta)
{
    *Ialpha = Ia;
    *Ibeta = (Ib - Ic) * INV_SQRT3;
}

float _normalizeAngle(float angle)
{
    const uint32_t el_counts_per_rev = 16384u * 7u;
    const float rad_per_count = _2PI / (float)el_counts_per_rev;
    int32_t el = (int32_t)(angle / rad_per_count);

    el %= (int32_t)el_counts_per_rev;
    if (el < 0) {
        el += (int32_t)el_counts_per_rev;
    }
    return (float)el * rad_per_count;
}

int cct = 0;

void setPhaseVoltage(TIM_HandleTypeDef *htim, float Uq, float Ud, float angle_el)
{
    float Uref;
    float T1, T2, T0;
    float Ta, Tb, Tc;
    float t0_half;
    int sector;
    float angle_ref;
    float theta;

    if (Ud == 0.0f) {
        float uq_abs = (Uq >= 0.0f) ? Uq : -Uq;

        Uref = uq_abs * INV_VBUS;
        if (Uref > 0.577f) {
            Uref = 0.577f;
        }

        angle_ref = angle_el + _PI_2;
        if (Uq < 0.0f) {
            angle_ref += _PI;
        }
        if (angle_ref >= _2PI) {
            angle_ref -= _2PI;
        }

        sector = (int)(angle_ref * INV_PI3);
        sector = (sector % 6) + 1;

        theta = angle_ref - (float)(sector - 1) * _PI_3;
        svpwm_t1_t2_from_theta(theta, Uref, &T1, &T2);
    } else {
        float U_alpha, U_beta;
        float sin_val = arm_sin_f32(angle_el);
        float cos_val = arm_cos_f32(angle_el);

        U_alpha = Ud * cos_val - Uq * sin_val;
        U_beta = Ud * sin_val + Uq * cos_val;

        Uref = sqrtf(U_alpha * U_alpha + U_beta * U_beta) * INV_VBUS;
        if (Uref > 1.0f) {
            Uref = 1.0f;
        } else if (Uref > 0.577f) {
            Uref = 0.577f + (Uref - 0.577f) * 0.8f;
        }

        angle_ref = atan2f(U_beta, U_alpha);
        if (angle_ref < 0.0f) {
            angle_ref += _2PI;
        }

        sector = (int)(angle_ref * INV_PI3);
        sector = (sector % 6) + 1;

        theta = angle_ref - (float)(sector - 1) * _PI_3;
        svpwm_t1_t2_from_theta(theta, Uref, &T1, &T2);
    }

    T0 = 1.0f - T1 - T2;
#if SVPWM_ALLOW_OVERMOD
    if (T0 < 0.0f) {
        float sum = T1 + T2;

        T0 = 0.0f;
        if (sum > 0.0f) {
            T1 /= sum;
            T2 /= sum;
        }
    }
#else
    if (T0 < 0.0f) {
        T0 = 0.0f;
    }
#endif

    t0_half = T0 * HALF_F;

    switch (sector) {
        case 1:
            Ta = T1 + T2 + t0_half;
            Tb = T2 + t0_half;
            Tc = t0_half;
            break;

        case 2:
            Ta = T1 + t0_half;
            Tb = T1 + T2 + t0_half;
            Tc = t0_half;
            break;

        case 3:
            Ta = t0_half;
            Tb = T1 + T2 + t0_half;
            Tc = T2 + t0_half;
            break;

        case 4:
            Ta = t0_half;
            Tb = T1 + t0_half;
            Tc = T1 + T2 + t0_half;
            break;

        case 5:
            Ta = T2 + t0_half;
            Tb = t0_half;
            Tc = T1 + T2 + t0_half;
            break;

        case 6:
            Ta = T1 + T2 + t0_half;
            Tb = t0_half;
            Tc = T1 + t0_half;
            break;

        default:
            Ta = Tb = Tc = 0.5f;
            break;
    }

    {
        uint16_t pwm_a = (uint16_t)(Ta * (float)PWM_Period);
        uint16_t pwm_b = (uint16_t)(Tb * (float)PWM_Period);
        uint16_t pwm_c = (uint16_t)(Tc * (float)PWM_Period);

        htim->Instance->CCR1 = pwm_a;
        htim->Instance->CCR2 = pwm_b;
        htim->Instance->CCR3 = pwm_c;
    }
}
