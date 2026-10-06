/**
 * @file hfi_pub.c
 * @date 2026-10-06
 * @brief HFI 与 SMO 发布角：切出去、退回来、关注入。
 *
 * 未发布用低速 ω 出门限。已发布用 SMO ω 决定注入和退回，避免 HFI 积分钉死。
 * 节拍限制见 hfi_pub.h 文件头，这里不重复。
 *
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
 */
#include "observer/hfi_pub.h"
#include "observer/obs_cfg.h"
#include "motor_math.h"
#include "observer/observer_composite.h"

/** 0=仍发布 HFI 角；1=已切到 SMO。 */
static uint8_t s_hfi_pub_smo;
/** 角融合系数 [0,1]。到 1 后 Park 用 SMO 角。 */
static float s_hfi_pub_a;
/** 发布给 Park 的电角，单位 rad。 */
static float s_hfi_pub_theta;
/** 出门限连续满足的拍数。满约 100 ms 才切出去。 */
static uint16_t s_hfi_pub_qual;
/** 退门限连续满足的拍数。 */
static uint16_t s_hfi_pub_dn;
/** SMO 电角速度低通，单位 rad/s。 */
static float s_smo_w_lp;
/** 1=低通已经用第一帧对齐。 */
static uint8_t s_smo_w_init;
/** 1=注入开着。 */
static uint8_t s_inj_on = 1u;
/** 发布阶段：0 HFI，0.5 融合，1 SMO 仍注入，2 SMO 注入关。 */
static float s_pub_ss;
/** 低通后的 SMO 机械转速，单位 rpm。 */
static float s_pub_smo_rpm;

/* SMO 转速做 5 ms 低通再拿来比。差在 25° 和 50 rpm 里连续 100 ms 才交。 */
/**
 * @brief 按转速和角差决定发布角、融合和注入。
 * @param theta_smo SMO 电角，单位 rad。
 * @param omega_el SMO 电角速度，单位 rad/s。
 * @param dth wrap(θ_smo − θ̂_hfi)，单位 rad。
 */
void hfi_pub_step(float theta_smo, float omega_el, float dth)
{
    const float rpm_scale = 60.0f / (6.28318530718f * (float)OBS_POLE_PAIRS);
    const float ang_ok = 0.436332f;
    const float ang_snap = 0.174533f;
    const float hfi_rpm = motor_absf(observer_lo_src()->get_omega() * rpm_scale);
    float smo_rpm;
    float dw;
    float gate_rpm;

    if (s_smo_w_init == 0u) {
        s_smo_w_lp = omega_el;
        s_smo_w_init = 1u;
    } else {
        s_smo_w_lp += 0.01f * (omega_el - s_smo_w_lp);
    }
    smo_rpm = s_smo_w_lp * rpm_scale;
    dw = smo_rpm - (observer_lo_src()->get_omega() * rpm_scale);
    s_pub_smo_rpm = smo_rpm;
    /* 未发布：HFI ω 决定何时切出去。已发布：SMO ω 决定注入/退回，避免积分钉死。 */
    gate_rpm = (s_hfi_pub_smo != 0u) ? motor_absf(smo_rpm) : hfi_rpm;
    if (observer_lo_src()->running() == 0u) {
        s_hfi_pub_smo = 0u;
        s_hfi_pub_a = 0.0f;
        s_hfi_pub_qual = 0u;
        s_hfi_pub_dn = 0u;
        s_inj_on = 1u;
        observer_lo_set_inj_scale(1.0f);
        observer_lo_set_iq_auth_hold(0u);
        s_pub_ss = 0.0f;
        return;
    }
    if (s_hfi_pub_smo == 0u) {
        if ((gate_rpm >= M1_HFI_PUB_UP_RPM) &&
            (motor_absf(dth) < ang_ok) &&
            (motor_absf(dw) < 50.0f)) {
            if (s_hfi_pub_qual < 2000u) {
                s_hfi_pub_qual++;
            }
            if (s_hfi_pub_qual >= 2000u) {
                s_hfi_pub_smo = 1u;
                s_hfi_pub_a = (motor_absf(dth) < ang_snap) ? 1.0f : 0.0f;
            }
        } else {
            s_hfi_pub_qual = 0u;
        }
    }
    if (s_hfi_pub_smo != 0u) {
        const float hat = observer_lo_src()->get_theta();
        const float live = motor_wrap_pi(theta_smo - hat);

        if (s_hfi_pub_a < 1.0f) {
            s_hfi_pub_a += (1.0f / 400.0f);
            if (s_hfi_pub_a > 1.0f) {
                s_hfi_pub_a = 1.0f;
            }
        }
        s_hfi_pub_theta = motor_wrap_pi(hat + (s_hfi_pub_a * live));
        if (s_hfi_pub_a >= 1.0f) {
            s_hfi_pub_theta = theta_smo;
            observer_lo_seed_hat(theta_smo, s_smo_w_lp);
        }
        gate_rpm = motor_absf(smo_rpm);
        if (gate_rpm <= M1_HFI_PUB_DOWN_RPM) {
            if (s_hfi_pub_dn < 2000u) {
                s_hfi_pub_dn++;
            }
            if (s_hfi_pub_dn >= 2000u) {
                s_hfi_pub_smo = 0u;
                s_hfi_pub_a = 0.0f;
                s_hfi_pub_qual = 0u;
                s_hfi_pub_dn = 0u;
            }
        } else {
            s_hfi_pub_dn = 0u;
        }
    }
    /* 还在 HFI 上时不关注入。SMO 已发布且 SMO ω 到 INJ_OFF 才关。 */
    if ((s_hfi_pub_smo != 0u) && (s_hfi_pub_a >= 1.0f) &&
        (gate_rpm >= M1_HFI_PUB_INJ_OFF_RPM)) {
        s_inj_on = 0u;
    } else if ((s_hfi_pub_smo == 0u) || (gate_rpm <= M1_HFI_PUB_INJ_ON_RPM)) {
        s_inj_on = 1u;
    }
    observer_lo_set_inj_scale((s_inj_on != 0u) ? 1.0f : 0.0f);
    observer_lo_set_iq_auth_hold((s_inj_on != 0u) ? 0u : 1u);
    if (s_hfi_pub_smo == 0u) {
        s_pub_ss = 0.0f;
    } else if (s_hfi_pub_a < 1.0f) {
        s_pub_ss = 0.5f;
    } else if (s_inj_on != 0u) {
        s_pub_ss = 1.0f;
    } else {
        s_pub_ss = 2.0f;
    }
}

/**
 * @brief 1=已经切到 SMO 发布。
 */
uint8_t hfi_pub_smo_active(void)
{
    return s_hfi_pub_smo;
}

float hfi_pub_theta(void)
{
    return s_hfi_pub_theta;
}

/**
 * @brief 读发布阶段：0 / 0.5 / 1 / 2。
 */
float hfi_pub_ss(void)
{
    return s_pub_ss;
}

float hfi_pub_smo_rpm(void)
{
    return s_pub_smo_rpm;
}
