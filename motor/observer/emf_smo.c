/**
 * @file emf_smo.c
 * @date 2026-10-06
 * @brief 电流滑模观测器。系数在 init 预计算。
 *
 * 默认不进 Park。节拍限制见 emf_smo.h 文件头。
 *
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
 */

#include "emf_smo.h"

#include <stddef.h>

#include "motor_math.h"
#include "observer/obs_cfg.h"
#include "obs_num.h"

#if M1_EMF_SMO_ENABLE

#define EMF_SMO_PI       3.14159265358979323846f
#define EMF_SMO_TWO_PI   6.28318530717958647692f

/** 离散电流方程系数 a。 */
static float s_disc_a;
/** 离散电流方程系数 b。 */
static float s_disc_b;
/** e-LPF 系数。 */
static float s_lpf_alpha;
/** 当前 LPF 截止频率，单位 Hz。 */
static float s_lpf_hz;
/** LPF 用的节拍，单位 s。 */
static float s_lpf_ts;
/** 机械 rpm 到电角速度的系数。 */
static float s_rpm_to_we;
/** 饱和函数的 1/ε。 */
static float s_inv_sat;
/** |rpm| 到电频率 Hz 的系数：极对数/60。 */
static float s_rpm_to_fe;
/** 1=系数已经算过。 */
static uint8_t s_coeff_ready;
static float emf_smo_wrap_pi(float x)
{
    if (x > EMF_SMO_PI) {
        x -= EMF_SMO_TWO_PI;
    } else if (x < -EMF_SMO_PI) {
        x += EMF_SMO_TWO_PI;
    }
    return x;
}

static float emf_smo_sat_fast(float x)
{
    x *= s_inv_sat;
    if (x > 1.0f) {
        return 1.0f;
    }
    if (x < -1.0f) {
        return -1.0f;
    }
    return x;
}

/** exp(-x)，x 为小正数。低通系数用，不进 libm。 */
static float emf_smo_exp_neg(float x)
{
    float term = 1.0f;
    float sum = 1.0f;
    int i;

    if (x < 0.0f) {
        x = -x;
    }
    if (x > 12.0f) {
        return 0.0f;
    }
    for (i = 1; i <= 8; i++) {
        term *= -x / (float)i;
        sum += term;
    }
    if (sum < 0.0f) {
        sum = 0.0f;
    }
    return sum;
}

static float emf_smo_alpha_from_hz(float fc_hz, float ts)
{
    float a = 1.0f - emf_smo_exp_neg(EMF_SMO_TWO_PI * fc_hz * ts);
    if (a > 1.0f) {
        a = 1.0f;
    }
    if (a < 0.0f) {
        a = 0.0f;
    }
    return a;
}

static void emf_smo_coeff_init(void)
{
    const float ts = OBS_CTRL_TS_S;
    const float r = g_obs_num.smo_r_ohm;
    const float l = g_obs_num.smo_l_h;

    s_disc_a = emf_smo_exp_neg(r * ts / l);
    s_disc_b = (1.0f - s_disc_a) / r;
    s_lpf_ts = ts;
#if M1_EMF_SMO_LPF_ENABLE
#if M1_EMF_SMO_LPF_LINEAR_ENABLE
    s_lpf_hz = g_obs_num.smo_lpf_fc_min_hz;
    s_lpf_alpha = emf_smo_alpha_from_hz(s_lpf_hz, ts);
#else
    s_lpf_hz = g_obs_num.smo_lpf_hz;
    s_lpf_alpha = emf_smo_alpha_from_hz(g_obs_num.smo_lpf_hz, ts);
#endif
#else
    s_lpf_hz = 0.0f;
    s_lpf_alpha = 1.0f;
#endif
    s_rpm_to_we = (EMF_SMO_TWO_PI / 60.0f) * (float)OBS_POLE_PAIRS;
    s_rpm_to_fe = ((float)OBS_POLE_PAIRS) / 60.0f;
    s_inv_sat = 1.0f / ((g_obs_num.smo_sat_a > 1.0e-6f) ? g_obs_num.smo_sat_a : 1.0e-6f);
    s_coeff_ready = 1u;
}

/**
 * @brief 预计算系数并复位状态。
 * @param o 观测器。不可为 NULL。
 */
void emf_smo_init(emf_smo_t *o)
{
    if (s_coeff_ready == 0u) {
        emf_smo_coeff_init();
    }
    emf_smo_reset(o);
}

/**
 * @brief 清估计电流和反电势。保留系数。
 * @param o 观测器。不可为 NULL。
 */
void emf_smo_reset(emf_smo_t *o)
{
    if (o == NULL) {
        return;
    }
    o->ihat_alpha = 0.0f;
    o->ihat_beta = 0.0f;
    o->primed = 0u;
    o->u_alpha = 0.0f;
    o->u_beta = 0.0f;
    o->e_alpha = 0.0f;
    o->e_beta = 0.0f;
    o->emag = 0.0f;
    o->theta_hat = 0.0f;
    o->theta_err = 0.0f;
    o->omega_el = 0.0f;
#if M1_EMF_SMO_LPF_LINEAR_ENABLE
    s_lpf_hz = g_obs_num.smo_lpf_fc_min_hz;
    s_lpf_alpha = emf_smo_alpha_from_hz(s_lpf_hz, s_lpf_ts);
#endif
}

/**
 * @brief 按机械转速改 e-LPF 截止频率。
 * @param omega_mech_rpm 机械转速，单位 rpm。
 * @return 分档时为档号；线性调度返回 0。
 */
uint8_t emf_smo_lpf_sched_update(float omega_mech_rpm)
{
#if M1_EMF_SMO_LPF_ENABLE && M1_EMF_SMO_LPF_LINEAR_ENABLE
    float rpm;
    float fe;
    float fc;
    float fc_q;

    if (s_coeff_ready == 0u) {
        emf_smo_coeff_init();
    }

    rpm = omega_mech_rpm;
    if (rpm < 0.0f) {
        rpm = -rpm;
    }
    /* fc = clip(k * fe, min, max), fe = |n| * p / 60 */
    fe = rpm * s_rpm_to_fe;
    fc = g_obs_num.smo_lpf_k * fe;
    if (fc < g_obs_num.smo_lpf_fc_min_hz) {
        fc = g_obs_num.smo_lpf_fc_min_hz;
    } else if (fc > g_obs_num.smo_lpf_fc_max_hz) {
        fc = g_obs_num.smo_lpf_fc_max_hz;
    }
    /* 量化到 1 Hz，避免每拍 expf */
    fc_q = (float)((int)(fc + 0.5f));
    if (fc_q != s_lpf_hz) {
        s_lpf_hz = fc_q;
        s_lpf_alpha = emf_smo_alpha_from_hz(fc_q, s_lpf_ts);
    }
    return 0u;
#else
    (void)omega_mech_rpm;
    return 0u;
#endif
}

/**
 * @brief 当前 LPF 截止频率，单位 Hz。
 */
float emf_smo_get_lpf_hz(void)
{
#if M1_EMF_SMO_ENABLE
    if (s_coeff_ready == 0u) {
        emf_smo_coeff_init();
    }
    return s_lpf_hz;
#else
    return 0.0f;
#endif
}

/**
 * @brief 用 iαβ、uαβ 推进一步。
 * @param o 观测器。不可为 NULL。
 * @param i_alpha α 电流，单位 A。
 * @param i_beta β 电流，单位 A。
 * @param u_alpha α 电压，单位 V。
 * @param u_beta β 电压，单位 V。
 * @param theta_enc 对照电角，单位 rad。
 * @param omega_mech_rpm 机械转速，单位 rpm。
 */
void emf_smo_update(emf_smo_t *o,
                    float i_alpha, float i_beta,
                    float u_alpha, float u_beta,
                    float theta_enc,
                    float omega_mech_rpm)
{
    float za;
    float zb;
    const float k = g_obs_num.smo_k;

    if (o == NULL) {
        return;
    }
    if (s_coeff_ready == 0u) {
        emf_smo_coeff_init();
    }

    o->u_alpha = u_alpha;
    o->u_beta = u_beta;

    if (o->primed == 0u) {
        o->ihat_alpha = i_alpha;
        o->ihat_beta = i_beta;
        o->primed = 1u;
        o->e_alpha = 0.0f;
        o->e_beta = 0.0f;
        o->emag = 0.0f;
        o->theta_hat = theta_enc;
        o->theta_err = 0.0f;
        o->omega_el = omega_mech_rpm * s_rpm_to_we;
        return;
    }

    za = k * emf_smo_sat_fast(o->ihat_alpha - i_alpha);
    zb = k * emf_smo_sat_fast(o->ihat_beta - i_beta);

    o->ihat_alpha = s_disc_a * o->ihat_alpha + s_disc_b * (u_alpha - za);
    o->ihat_beta = s_disc_a * o->ihat_beta + s_disc_b * (u_beta - zb);

#if M1_EMF_SMO_LPF_ENABLE
    o->e_alpha += s_lpf_alpha * (za - o->e_alpha);
    o->e_beta += s_lpf_alpha * (zb - o->e_beta);
#else
    o->e_alpha = za;
    o->e_beta = zb;
#endif

#if M1_EMF_SMO_ATAN_ENABLE
    /* 仅对照/遥测路径要 |e| 与 θ̂；PLL 吃 eαβ 时整段跳过 */
#if M1_EMF_EMAG_FAST_ENABLE
    o->emag = motor_emag_approx(o->e_alpha, o->e_beta);
#else
    o->emag = sqrtf(o->e_alpha * o->e_alpha + o->e_beta * o->e_beta);
#endif
    {
        float th_hat;

        th_hat = atan2f(-o->e_alpha, o->e_beta) - g_obs_num.smo_theta_off_rad;
        o->theta_hat = emf_smo_wrap_pi(th_hat);
        o->theta_err = emf_smo_wrap_pi(o->theta_hat - theta_enc);
    }
#else
    (void)theta_enc;
#endif
    o->omega_el = omega_mech_rpm * s_rpm_to_we;
}

#else /* !M1_EMF_SMO_ENABLE */

void emf_smo_init(emf_smo_t *o)
{
    (void)o;
}

void emf_smo_reset(emf_smo_t *o)
{
    (void)o;
}

uint8_t emf_smo_lpf_sched_update(float omega_mech_rpm)
{
    (void)omega_mech_rpm;
    return 0u;
}

float emf_smo_get_lpf_hz(void)
{
    return 0.0f;
}

void emf_smo_update(emf_smo_t *o,
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

#endif /* M1_EMF_SMO_ENABLE */
