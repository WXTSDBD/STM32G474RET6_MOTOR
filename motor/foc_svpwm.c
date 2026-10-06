/**
 * @file foc_svpwm.c
 * @date 2026-10-06
 * @brief Clarke、Park、SVPWM 和死区占空比补偿。
 *
 * 节拍限制见 foc_svpwm.h 文件头。
 *
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
 */

#include "foc_svpwm.h"

#include "tim.h"

#include "dbg_monitor.h"
#include "deadband.h"
#include "motor_params_m1.h"
#include "motor_phase_binding.h"
#include "motor_trig.h"

#include <math.h>

#define voltage_power_supply 24
#define PWM_Period 3999
#define _PI 3.14159265359f
#define _PI_2 1.57079632679f
#define _PI_3 1.0471975512f
#define _2PI 6.28318530718f
#define _SQRT3 1.73205080757f
#define INV_SQRT3 (1.0f / _SQRT3)
#define INV_VBUS   (1.0f / (float)voltage_power_supply)
#define INV_PI3    (1.0f / _PI_3)
#define HALF_F     0.5f
#define SIN_PI3    0.86602540378f

#ifndef SVPWM_ALLOW_OVERMOD
#define SVPWM_ALLOW_OVERMOD 0
#endif

/**
 * @brief 由扇区内角和电压幅值算两矢量作用时间。
 */
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

/**
 * @brief αβ 到 dq。Id/Iq 指针不可为 NULL。
 */
void Park_Transform(float Ialpha, float Ibeta, float theta, float *Id, float *Iq)
{
    float sin_val;
    float cos_val;

    motor_trig_sincos(theta, &cos_val, &sin_val);
    *Id = Ialpha * cos_val + Ibeta * sin_val;
    *Iq = -Ialpha * sin_val + Ibeta * cos_val;
}

/**
 * @brief αβ 到 dq，正余弦由调用方提供，避免再算一次。
 */
void Park_Transform_sc(float Ialpha, float Ibeta,
                       float sin_el, float cos_el,
                       float *Id, float *Iq)
{
    *Id = Ialpha * cos_el + Ibeta * sin_el;
    *Iq = -Ialpha * sin_el + Ibeta * cos_el;
}

/**
 * @brief dq 到 αβ。
 */
void Anti_Park_Transform(float mod_d, float mod_q, float theta, float *mod_alpha, float *mod_beta)
{
    float sin_val;
    float cos_val;

    motor_trig_sincos(theta, &cos_val, &sin_val);
    *mod_alpha = mod_d * cos_val - mod_q * sin_val;
    *mod_beta = mod_d * sin_val + mod_q * cos_val;
}

/**
 * @brief dq 到 αβ，正余弦由调用方提供。
 */
void Anti_Park_Transform_sc(float mod_d, float mod_q,
                            float sin_el, float cos_el,
                            float *mod_alpha, float *mod_beta)
{
    *mod_alpha = mod_d * cos_el - mod_q * sin_el;
    *mod_beta = mod_d * sin_el + mod_q * cos_el;
}

/**
 * @brief 三相到 αβ。Iα=Ia，Iβ=(Ib−Ic)/√3。
 */
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

static void svpwm_write_ccr(TIM_HandleTypeDef *htim, float Ta, float Tb, float Tc)
{
    motor_phase_binding_write_ccr(htim, Ta, Tb, Tc, PWM_Period);
}

static float svpwm_duty_dev(float ta, float tb, float tc)
{
    float da = ta - HALF_F;

    if (da < 0.0f) {
        da = -da;
    }
    {
        float db = tb - HALF_F;

        if (db < 0.0f) {
            db = -db;
        }
        if (db > da) {
            da = db;
        }
    }
    {
        float dc = tc - HALF_F;

        if (dc < 0.0f) {
            dc = -dc;
        }
        if (dc > da) {
            da = dc;
        }
    }
    return da;
}

/**
 * @brief 由 dq 电压和电角判断扇区，1..6。
 */
int svpwm_sector_from_uq_ud(float Uq, float Ud, float angle_el)
{
    float angle_ref;
    int sector;

    if (Ud == 0.0f) {
        angle_ref = angle_el + _PI_2;
        if (Uq < 0.0f) {
            angle_ref += _PI;
        }
        if (angle_ref >= _2PI) {
            angle_ref -= _2PI;
        }
    } else {
        float U_alpha;
        float U_beta;
        float sin_val;
        float cos_val;

        motor_trig_sincos(angle_el, &cos_val, &sin_val);
        U_alpha = Ud * cos_val - Uq * sin_val;
        U_beta = Ud * sin_val + Uq * cos_val;
        angle_ref = atan2f(U_beta, U_alpha);
        if (angle_ref < 0.0f) {
            angle_ref += _2PI;
        }
    }

    sector = (int)(angle_ref * INV_PI3);
    sector = (sector % 6) + 1;
    return sector;
}

static void setPhaseVoltage_core(TIM_HandleTypeDef *htim,
                                 float Uq, float Ud, float angle_el,
                                 float ia, float ib, float ic,
                                 float id_dq, float iq_dq,
                                 int apply_deadband)
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

        sector = svpwm_sector_from_uq_ud(Uq, Ud, angle_el);
        angle_ref = angle_el + _PI_2;
        if (Uq < 0.0f) {
            angle_ref += _PI;
        }
        if (angle_ref >= _2PI) {
            angle_ref -= _2PI;
        }

        theta = angle_ref - (float)(sector - 1) * _PI_3;
        svpwm_t1_t2_from_theta(theta, Uref, &T1, &T2);
    } else {
        float U_alpha;
        float U_beta;
        float sin_val;
        float cos_val;

        motor_trig_sincos(angle_el, &cos_val, &sin_val);
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

        sector = svpwm_sector_from_uq_ud(Uq, Ud, angle_el);

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

    if (apply_deadband) {
        deadband_apply_duty(ia, ib, ic, angle_el, id_dq, iq_dq, &Ta, &Tb, &Tc);
    }

    dbg.foc_duty_ta = Ta;
    dbg.foc_duty_tb = Tb;
    dbg.foc_duty_tc = Tc;
    dbg.foc_svpwm_uref = Uref;
    dbg.foc_svpwm_duty_dev = svpwm_duty_dev(Ta, Tb, Tc);
    {
        float dab = Ta - Tb;

        if (dab < 0.0f) {
            dab = -dab;
        }
        dbg.foc_svpwm_duty_ab = dab;
    }
    dbg.foc_svpwm_sector = (uint8_t)sector;
    {
        float sin_el;
        float cos_el;
        float va;
        float vb;
        float vc;
        float valpha;
        float vbeta;

        motor_trig_sincos(angle_el, &cos_el, &sin_el);
        va = (Ta - HALF_F) * M1_VBUS_V;
        vb = (Tb - HALF_F) * M1_VBUS_V;
        vc = (Tc - HALF_F) * M1_VBUS_V;
        valpha = va;
        vbeta = (vb - vc) * INV_SQRT3;
        dbg.foc_vd_est = valpha * cos_el + vbeta * sin_el;
        dbg.foc_vq_est = -valpha * sin_el + vbeta * cos_el;
    }

    svpwm_write_ccr(htim, Ta, Tb, Tc);
}

/**
 * @brief 写出三相 PWM。不带电流重构补偿。
 * @param axis 轴。不可为 NULL。
 * @param Uq q 轴电压，单位 V。
 * @param Ud d 轴电压，单位 V。
 * @param angle_el 电角，单位 rad。
 */
void foc_svpwm_apply(bsp_axis_t *axis, float Uq, float Ud, float angle_el)
{
    if (axis == NULL || axis->pwm == NULL || axis->pwm->hw == NULL) {
        return;
    }
    setPhaseVoltage_core((TIM_HandleTypeDef *)axis->pwm->hw,
                         Uq, Ud, angle_el,
                         0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0);
}

/**
 * @brief 写出三相 PWM，并可按相电流做死区补偿。
 */
void foc_svpwm_apply_abc(bsp_axis_t *axis,
                         float Uq, float Ud, float angle_el,
                         float ia, float ib, float ic,
                         float id_dq, float iq_dq)
{
    if (axis == NULL || axis->pwm == NULL || axis->pwm->hw == NULL) {
        return;
    }
    setPhaseVoltage_core((TIM_HandleTypeDef *)axis->pwm->hw,
                         Uq, Ud, angle_el, ia, ib, ic, id_dq, iq_dq, 1);
}
