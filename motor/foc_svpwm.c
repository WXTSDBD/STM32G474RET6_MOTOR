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

#include <stddef.h>

#include "dbg_monitor.h"
#include "deadband.h"
#include "motor_cfg.h"
#include "motor_params_m1.h"
#include "motor_phase_binding.h"
#include "motor_trig.h"
#include "pwm_port.h"

#define INV_SQRT3  0.57735026919f
#define TWO_BY_SQRT3 (2.0f * INV_SQRT3)
#define HALF_F     0.5f

/**
 * @brief 占空比限制在 0..1。
 */
static float svpwm_sat01(float x)
{
    if (x < 0.0f) {
        return 0.0f;
    }
    if (x > 1.0f) {
        return 1.0f;
    }
    return x;
}

/**
 * @brief 由 αβ 电压比较得到扇区，1..6。与 VESC foc_svm 同一组判断。
 */
static int svpwm_sector_ab(float ua, float ub)
{
    if (ub >= 0.0f) {
        if (ua >= 0.0f) {
            if (INV_SQRT3 * ub > ua) {
                return 2;
            }
            return 1;
        }
        if (-INV_SQRT3 * ub > ua) {
            return 3;
        }
        return 2;
    }
    if (ua >= 0.0f) {
        if (-INV_SQRT3 * ub > ua) {
            return 5;
        }
        return 6;
    }
    if (INV_SQRT3 * ub > ua) {
        return 4;
    }
    return 5;
}

/**
 * @brief 由归一化 αβ 调制比写三相高侧占空比。
 * @param ma α 调制比。
 * @param mb β 调制比。
 */
static void svpwm_duty_from_mab(float ma, float mb,
                                float *ta, float *tb, float *tc,
                                int *sector_out)
{
    const int sector = svpwm_sector_ab(ma, mb);
    float t1;
    float t2;
    float a;
    float b;
    float c;

    switch (sector) {
    case 1:
        t1 = ma - INV_SQRT3 * mb;
        t2 = TWO_BY_SQRT3 * mb;
        a = (1.0f + t1 + t2) * HALF_F;
        b = a - t1;
        c = b - t2;
        break;
    case 2:
        t1 = ma + INV_SQRT3 * mb;
        t2 = -ma + INV_SQRT3 * mb;
        b = (1.0f + t1 + t2) * HALF_F;
        a = b - t2;
        c = a - t1;
        break;
    case 3:
        t1 = TWO_BY_SQRT3 * mb;
        t2 = -ma - INV_SQRT3 * mb;
        b = (1.0f + t1 + t2) * HALF_F;
        c = b - t1;
        a = c - t2;
        break;
    case 4:
        t1 = -ma + INV_SQRT3 * mb;
        t2 = -TWO_BY_SQRT3 * mb;
        c = (1.0f + t1 + t2) * HALF_F;
        b = c - t2;
        a = b - t1;
        break;
    case 5:
        t1 = -ma - INV_SQRT3 * mb;
        t2 = ma - INV_SQRT3 * mb;
        c = (1.0f + t1 + t2) * HALF_F;
        a = c - t1;
        b = a - t2;
        break;
    case 6:
    default:
        t1 = -TWO_BY_SQRT3 * mb;
        t2 = ma + INV_SQRT3 * mb;
        a = (1.0f + t1 + t2) * HALF_F;
        c = a - t2;
        b = c - t1;
        break;
    }
    *ta = svpwm_sat01(a);
    *tb = svpwm_sat01(b);
    *tc = svpwm_sat01(c);
    *sector_out = sector;
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

static void svpwm_write_ccr(pwm_port_t *port, float Ta, float Tb, float Tc)
{
    /* 唯一来源：板级 port->arr_counts；忘填不得静默回落宏。 */
    if ((port == NULL) || (port->arr_counts == 0u)) {
        return;
    }
    motor_phase_binding_write_ccr(port, Ta, Tb, Tc, port->arr_counts);
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
 * @brief 由 dq 电压和电角判断扇区，1..6。比较 αβ，不用 atan2。
 */
int svpwm_sector_from_uq_ud(float Uq, float Ud, float angle_el)
{
    float sin_el;
    float cos_el;
    float ua;
    float ub;

    motor_trig_sincos(angle_el, &cos_el, &sin_el);
    ua = Ud * cos_el - Uq * sin_el;
    ub = Ud * sin_el + Uq * cos_el;
    return svpwm_sector_ab(ua, ub);
}

/**
 * @brief 扇区比较调制。αβ 用本拍正余弦，占空比钳在 0..1。
 * @note sin_el、cos_el 与 angle_el 同一拍。
 */
static void setPhaseVoltage_core(pwm_port_t *port,
                                 float Uq, float Ud, float angle_el,
                                 float sin_el, float cos_el,
                                 float ia, float ib, float ic,
                                 float id_dq, float iq_dq,
                                 int apply_deadband)
{
    float Ta;
    float Tb;
    float Tc;
    float ma;
    float mb;
    int sector;
    const float vbus = g_m1_motor_cfg.vbus_v;
    const float k = 1.5f / vbus;

    ma = k * (Ud * cos_el - Uq * sin_el);
    mb = k * (Ud * sin_el + Uq * cos_el);
    svpwm_duty_from_mab(ma, mb, &Ta, &Tb, &Tc, &sector);

    if (apply_deadband) {
        deadband_apply_duty(ia, ib, ic, angle_el, id_dq, iq_dq, &Ta, &Tb, &Tc);
    }

#if M1_DBG_ISR_MIRROR_SVPWM
    DBG.foc_duty_ta = Ta;
    DBG.foc_duty_tb = Tb;
    DBG.foc_duty_tc = Tc;
    {
        float ua = ma;
        float ub = mb;

        if (ua < 0.0f) {
            ua = -ua;
        }
        if (ub < 0.0f) {
            ub = -ub;
        }
        DBG.foc_svpwm_uref = (ua > ub) ? ua : ub;
    }
    DBG.foc_svpwm_duty_dev = svpwm_duty_dev(Ta, Tb, Tc);
    {
        float dab = Ta - Tb;

        if (dab < 0.0f) {
            dab = -dab;
        }
        DBG.foc_svpwm_duty_ab = dab;
    }
    DBG.foc_svpwm_sector = (uint8_t)sector;
    {
        float va = (Ta - HALF_F) * vbus;
        float vb = (Tb - HALF_F) * vbus;
        float vc = (Tc - HALF_F) * vbus;
        float valpha = va;
        float vbeta = (vb - vc) * INV_SQRT3;

        DBG.foc_vd_est = valpha * cos_el + vbeta * sin_el;
        DBG.foc_vq_est = -valpha * sin_el + vbeta * cos_el;
    }
#endif

    svpwm_write_ccr(port, Ta, Tb, Tc);
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
    float sin_el;
    float cos_el;

    if (axis == NULL || axis->pwm == NULL || axis->pwm->hw == NULL) {
        return;
    }
    motor_trig_sincos(angle_el, &cos_el, &sin_el);
    setPhaseVoltage_core(axis->pwm,
                         Uq, Ud, angle_el, sin_el, cos_el,
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
    float sin_el;
    float cos_el;

    if (axis == NULL || axis->pwm == NULL || axis->pwm->hw == NULL) {
        return;
    }
    motor_trig_sincos(angle_el, &cos_el, &sin_el);
    setPhaseVoltage_core(axis->pwm,
                         Uq, Ud, angle_el, sin_el, cos_el,
                         ia, ib, ic, id_dq, iq_dq,
                         M1_DEADBAND_ENABLE);
}

/**
 * @brief 与 apply_abc 相同，正余弦由本拍 Park 传入。
 */
void foc_svpwm_apply_abc_sc(bsp_axis_t *axis,
                            float Uq, float Ud, float angle_el,
                            float sin_el, float cos_el,
                            float ia, float ib, float ic,
                            float id_dq, float iq_dq)
{
    if (axis == NULL || axis->pwm == NULL || axis->pwm->hw == NULL) {
        return;
    }
    setPhaseVoltage_core(axis->pwm,
                         Uq, Ud, angle_el, sin_el, cos_el,
                         ia, ib, ic, id_dq, iq_dq,
                         M1_DEADBAND_ENABLE);
}
