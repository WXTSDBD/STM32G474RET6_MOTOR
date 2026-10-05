/**
 * @file hfi_pub.c
 * @brief P5: HFI helpers moved from motor_current.c (call order unchanged).
 */
#include "observer/hfi_pub.h"
#include "observer/obs_cfg.h"
#include "motor_math.h"
#include "observer/hfi_sqwave.h"
#if M1_HFI_GATE == 131
#include "dbg_monitor.h"
#endif

#if (M1_HFI_GATE == 131) || (M1_HFI_GATE == 138) || (M1_HFI_GATE == 141)
/* 0=发布 HFI，1=发布 SMO。旧交接状态机不参与。
 * 131：门槛跟速度指令。138/141：门槛跟实测转速的绝对值。 */
static uint8_t s_hfi_pub_smo;
static float s_hfi_pub_a;
static float s_hfi_pub_theta;
static uint16_t s_hfi_pub_qual;
static uint16_t s_hfi_pub_dn;
static float s_smo_w_lp;
static uint8_t s_smo_w_init;
static uint8_t s_inj_on = 1u;
static float s_pub_ss;
static float s_pub_smo_rpm;

/* SMO 转速做 5 ms 低通再拿来比。差在 25° 和 50 rpm 里连续 100 ms 才交。 */
void hfi_pub_step(float theta_smo, float omega_el, float dth)
{
    const float rpm_scale = 60.0f / (6.28318530718f * (float)OBS_POLE_PAIRS);
    const float ang_ok = 0.436332f;
    const float ang_snap = 0.174533f;
#if (M1_HFI_GATE == 138) || (M1_HFI_GATE == 141)
    const float gate_rpm = motor_absf(hfi_sqwave_get_pll_int_el() * rpm_scale);
#else
    const float gate_rpm = dbg.outer_omega_ref;
#endif
    float smo_rpm;
    float dw;

    if (s_smo_w_init == 0u) {
        s_smo_w_lp = omega_el;
        s_smo_w_init = 1u;
    } else {
        s_smo_w_lp += 0.01f * (omega_el - s_smo_w_lp);
    }
    smo_rpm = s_smo_w_lp * rpm_scale;
    dw = smo_rpm - (hfi_sqwave_get_pll_int_el() * rpm_scale);
    s_pub_smo_rpm = smo_rpm;
    if (hfi_sqwave_speed_run_active() == 0u) {
        s_hfi_pub_smo = 0u;
        s_hfi_pub_a = 0.0f;
        s_hfi_pub_qual = 0u;
        s_hfi_pub_dn = 0u;
        s_inj_on = 1u;
        hfi_sqwave_set_inj_scale(1.0f);
        hfi_sqwave_set_iq_auth_hold(0u);
        s_pub_ss = 0.0f;
        return;
    }
    if (s_hfi_pub_smo == 0u) {
        if ((gate_rpm >= 1300.0f) &&
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
        const float hat = hfi_sqwave_get_theta_hat();
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
            hfi_sqwave_seed_hat(theta_smo, s_smo_w_lp);
        }
        if (gate_rpm <= 1000.0f) {
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
    /* 还在 HFI 上时不关注入。SMO 已发布且门槛到 1500 才关。
     * 131 的门槛是指令。138/141 的门槛是实测转速绝对值。 */
    if ((s_hfi_pub_smo != 0u) && (s_hfi_pub_a >= 1.0f) &&
        (gate_rpm >= 1500.0f)) {
        s_inj_on = 0u;
    } else if ((s_hfi_pub_smo == 0u) || (gate_rpm <= 1400.0f)) {
        s_inj_on = 1u;
    }
    hfi_sqwave_set_inj_scale((s_inj_on != 0u) ? 1.0f : 0.0f);
    hfi_sqwave_set_iq_auth_hold((s_inj_on != 0u) ? 0u : 1u);
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

uint8_t hfi_pub_smo_active(void)
{
    return s_hfi_pub_smo;
}

float hfi_pub_theta(void)
{
    return s_hfi_pub_theta;
}

float hfi_pub_ss(void)
{
    return s_pub_ss;
}

float hfi_pub_smo_rpm(void)
{
    return s_pub_smo_rpm;
}

#else /* not 131/138/141 */

void hfi_pub_step(float theta_smo, float omega_el, float dth)
{
    (void)theta_smo;
    (void)omega_el;
    (void)dth;
}

uint8_t hfi_pub_smo_active(void)
{
    return 0u;
}

float hfi_pub_theta(void)
{
    return 0.0f;
}

float hfi_pub_ss(void)
{
    return 0.0f;
}

float hfi_pub_smo_rpm(void)
{
    return 0.0f;
}

#endif
