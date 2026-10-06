/**
 * @file emf_veq.c
 * @date 2026-10-06
 * @brief 电压方程反电势。系数在 init 预计算。
 *
 * 默认不进 Park。节拍限制见 emf_veq.h 文件头。
 *
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
 */

#include "emf_veq.h"

#include <math.h>
#include <stddef.h>

#include "observer/obs_cfg.h"

#ifndef M1_EMF_VEQ_ENABLE
#define M1_EMF_VEQ_ENABLE 0
#endif

#if M1_EMF_VEQ_ENABLE

#ifndef M1_EMF_VEQ_R_OHM
#define M1_EMF_VEQ_R_OHM            OBS_RS_OHM
#endif
#ifndef M1_EMF_VEQ_L_H
#define M1_EMF_VEQ_L_H              OBS_LD_H
#endif
#ifndef M1_EMF_VEQ_LPF_HZ
#define M1_EMF_VEQ_LPF_HZ           200.0f
#endif
#ifndef M1_EMF_VEQ_LPF_ENABLE
#define M1_EMF_VEQ_LPF_ENABLE       1
#endif
#ifndef M1_EMF_VEQ_THETA_OFF_RAD
#define M1_EMF_VEQ_THETA_OFF_RAD    (-0.4054f)
#endif
#ifndef M1_EMF_VEQ_PSI_WMIN
#define M1_EMF_VEQ_PSI_WMIN         20.0f
#endif

#define EMF_VEQ_PI       3.14159265358979323846f
#define EMF_VEQ_TWO_PI   6.28318530717958647692f

/** 与 M1_PI_KP 一样：固定 Ts 下一次算清 */
static float s_inv_ts;
static float s_lpf_alpha;
static float s_rpm_to_we;
static uint8_t s_coeff_ready;

static float emf_veq_wrap_pi(float x)
{
    if (x > EMF_VEQ_PI) {
        x -= EMF_VEQ_TWO_PI;
    } else if (x < -EMF_VEQ_PI) {
        x += EMF_VEQ_TWO_PI;
    }
    return x;
}

static void emf_veq_coeff_init(void)
{
    const float ts = OBS_CTRL_TS_S;

    s_inv_ts = 1.0f / ts;
#if M1_EMF_VEQ_LPF_ENABLE
    s_lpf_alpha = 1.0f - expf(-EMF_VEQ_TWO_PI * M1_EMF_VEQ_LPF_HZ * ts);
    if (s_lpf_alpha > 1.0f) {
        s_lpf_alpha = 1.0f;
    }
    if (s_lpf_alpha < 0.0f) {
        s_lpf_alpha = 0.0f;
    }
#else
    s_lpf_alpha = 1.0f;
#endif
    s_rpm_to_we = (EMF_VEQ_TWO_PI / 60.0f) * (float)OBS_POLE_PAIRS;
    s_coeff_ready = 1u;
}

/**
 * @brief 预计算系数并复位。
 * @param o 观测器。不可为 NULL。
 */
void emf_veq_init(emf_veq_t *o)
{
    if (s_coeff_ready == 0u) {
        emf_veq_coeff_init();
    }
    emf_veq_reset(o);
}

/**
 * @brief 清上一拍电流和反电势。保留系数。
 * @param o 观测器。不可为 NULL。
 */
void emf_veq_reset(emf_veq_t *o)
{
    if (o == NULL) {
        return;
    }
    o->i_alpha_prev = 0.0f;
    o->i_beta_prev = 0.0f;
    o->primed = 0u;
    o->u_alpha = 0.0f;
    o->u_beta = 0.0f;
    o->i_alpha = 0.0f;
    o->i_beta = 0.0f;
    o->e_alpha = 0.0f;
    o->e_beta = 0.0f;
    o->emag = 0.0f;
    o->theta_hat = 0.0f;
    o->theta_err = 0.0f;
    o->omega_el = 0.0f;
    o->psi_inst = 0.0f;
}

/**
 * @brief 用已反 Park 的 uαβ 和 iαβ 估计反电势。
 * @param o 观测器。不可为 NULL。
 * @param i_alpha α 电流，单位 A。
 * @param i_beta β 电流，单位 A。
 * @param u_alpha α 电压，单位 V。须与 FOC 同一组正余弦。
 * @param u_beta β 电压，单位 V。
 * @param theta_enc 对照电角，单位 rad。
 * @param omega_mech_rpm 机械转速，单位 rpm。
 */
void emf_veq_update(emf_veq_t *o,
                    float i_alpha, float i_beta,
                    float u_alpha, float u_beta,
                    float theta_enc,
                    float omega_mech_rpm)
{
    float dia;
    float dib;
    float ea_raw;
    float eb_raw;
    float we;
    float th_hat;
    const float r = M1_EMF_VEQ_R_OHM;
    const float l = M1_EMF_VEQ_L_H;

    if (o == NULL) {
        return;
    }
    if (s_coeff_ready == 0u) {
        emf_veq_coeff_init();
    }

    o->i_alpha = i_alpha;
    o->i_beta = i_beta;
    o->u_alpha = u_alpha;
    o->u_beta = u_beta;

    if (o->primed == 0u) {
        o->i_alpha_prev = i_alpha;
        o->i_beta_prev = i_beta;
        o->primed = 1u;
        o->e_alpha = 0.0f;
        o->e_beta = 0.0f;
        o->emag = 0.0f;
        o->theta_hat = theta_enc;
        o->theta_err = 0.0f;
        o->omega_el = omega_mech_rpm * s_rpm_to_we;
        o->psi_inst = 0.0f;
        return;
    }

    dia = (i_alpha - o->i_alpha_prev) * s_inv_ts;
    dib = (i_beta - o->i_beta_prev) * s_inv_ts;
    o->i_alpha_prev = i_alpha;
    o->i_beta_prev = i_beta;

    ea_raw = u_alpha - r * i_alpha - l * dia;
    eb_raw = u_beta - r * i_beta - l * dib;

#if M1_EMF_VEQ_LPF_ENABLE
    o->e_alpha += s_lpf_alpha * (ea_raw - o->e_alpha);
    o->e_beta += s_lpf_alpha * (eb_raw - o->e_beta);
#else
    o->e_alpha = ea_raw;
    o->e_beta = eb_raw;
#endif

    o->emag = sqrtf(o->e_alpha * o->e_alpha + o->e_beta * o->e_beta);
    th_hat = atan2f(-o->e_alpha, o->e_beta) - M1_EMF_VEQ_THETA_OFF_RAD;
    o->theta_hat = emf_veq_wrap_pi(th_hat);
    o->theta_err = emf_veq_wrap_pi(o->theta_hat - theta_enc);

    we = omega_mech_rpm * s_rpm_to_we;
    o->omega_el = we;
    if (fabsf(we) > M1_EMF_VEQ_PSI_WMIN) {
        o->psi_inst = o->emag / fabsf(we);
    }
}

#else /* !M1_EMF_VEQ_ENABLE */

void emf_veq_init(emf_veq_t *o)
{
    (void)o;
}

void emf_veq_reset(emf_veq_t *o)
{
    (void)o;
}

void emf_veq_update(emf_veq_t *o,
                    float i_alpha, float i_beta,
                    float u_alpha, float u_beta,
                    float theta_enc,
                    float omega_mech_rpm)
{
    (void)o;
    (void)i_alpha;
    (void)i_beta;
    (void)u_alpha;
    (void)u_beta;
    (void)theta_enc;
    (void)omega_mech_rpm;
}

#endif /* M1_EMF_VEQ_ENABLE */
