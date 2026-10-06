/**
 * @file observer_composite.c
 * @brief HFI Composite 聚合器：ops/inj、发布角、EMF-PLL 包装、低速槽。
 */
#include "observer/observer_composite.h"
#include "observer/obs_cfg.h"

#ifndef M1_HFI_ENABLE
#define M1_HFI_ENABLE 0
#endif

#if M1_HFI_ENABLE
#include "observer/hfi_sqwave.h"
#include "observer/obs_angle.h"
#include "observer/obs_inj.h"
#include "observer/hfi_pub.h"
#include "observer/hfi_spd_obs.h"
#include "observer/emf_pll.h"
#include <stddef.h>

_Static_assert((int)OBS_STAGE_IDLE   == (int)HFI_STAGE_IDLE,   "OBS_STAGE_IDLE");
_Static_assert((int)OBS_STAGE_MOVE   == (int)HFI_STAGE_MOVE,   "OBS_STAGE_MOVE");
_Static_assert((int)OBS_STAGE_SETTLE == (int)HFI_STAGE_SETTLE, "OBS_STAGE_SETTLE");
_Static_assert((int)OBS_STAGE_MEAS   == (int)HFI_STAGE_MEAS,   "OBS_STAGE_MEAS");
_Static_assert((int)OBS_STAGE_LOG    == (int)HFI_STAGE_LOG,    "OBS_STAGE_LOG");
_Static_assert((int)OBS_STAGE_DONE   == (int)HFI_STAGE_DONE,   "OBS_STAGE_DONE");
_Static_assert((int)OBS_STAGE_CRAWL  == (int)HFI_STAGE_CRAWL,  "OBS_STAGE_CRAWL");
_Static_assert((int)OBS_STAGE_RUN    == (int)HFI_STAGE_RUN,    "OBS_STAGE_RUN");
_Static_assert((int)OBS_LOCK_CAPTURE == (int)HFI_LOCK_CAPTURE, "OBS_LOCK_CAPTURE");
_Static_assert((int)OBS_LOCK_LOCKED  == (int)HFI_LOCK_LOCKED,  "OBS_LOCK_LOCKED");
_Static_assert((int)OBS_LOCK_FAULT   == (int)HFI_LOCK_FAULT,   "OBS_LOCK_FAULT");
_Static_assert(offsetof(observer_view_t, stage) == 0u, "view.stage");
_Static_assert(offsetof(observer_view_t, lock) > offsetof(observer_view_t, stage), "view.lock");
_Static_assert(offsetof(observer_view_t, theta_cmd) > offsetof(observer_view_t, lock), "view.theta_cmd");
_Static_assert(offsetof(observer_view_t, theta_hat) > offsetof(observer_view_t, theta_cmd), "view.theta_hat");
_Static_assert(offsetof(observer_view_t, theta_err) > offsetof(observer_view_t, theta_hat), "view.theta_err");
_Static_assert(offsetof(observer_view_t, eps) > offsetof(observer_view_t, theta_err), "view.eps");
_Static_assert(offsetof(observer_view_t, di_q) > offsetof(observer_view_t, eps), "view.di_q");
_Static_assert(offsetof(observer_view_t, di_d) > offsetof(observer_view_t, di_q), "view.di_d");
_Static_assert(offsetof(observer_view_t, x_raw) > offsetof(observer_view_t, di_d), "view.x_raw");
_Static_assert(offsetof(observer_view_t, y_raw) > offsetof(observer_view_t, x_raw), "view.y_raw");
_Static_assert(offsetof(observer_view_t, vh_sign) > offsetof(observer_view_t, y_raw), "view.vh_sign");
_Static_assert(offsetof(observer_view_t, eps_dead) > offsetof(observer_view_t, vh_sign), "view.eps_dead");
_Static_assert(offsetof(observer_view_t, pll_int_el) > offsetof(observer_view_t, eps_dead), "view.pll_int_el");
_Static_assert(offsetof(observer_view_t, omega_el) > offsetof(observer_view_t, pll_int_el), "view.omega_el");
_Static_assert(offsetof(observer_view_t, omega_trim_el) > offsetof(observer_view_t, omega_el), "view.omega_trim_el");
_Static_assert(offsetof(observer_view_t, ipd_phase) > offsetof(observer_view_t, omega_trim_el), "view.ipd_phase");
_Static_assert(offsetof(observer_view_t, ipd_pulse_ud) > offsetof(observer_view_t, ipd_phase), "view.ipd_pulse_ud");
_Static_assert(offsetof(observer_view_t, eps_d) > offsetof(observer_view_t, ipd_pulse_ud), "view.eps_d");
_Static_assert(offsetof(observer_view_t, qkick_verdict) > offsetof(observer_view_t, eps_d), "view.qkick_verdict");
_Static_assert(offsetof(observer_view_t, axis_flip_n) > offsetof(observer_view_t, qkick_verdict), "view.axis_flip_n");
_Static_assert(offsetof(observer_view_t, axis_ok) > offsetof(observer_view_t, axis_flip_n), "view.axis_ok");
_Static_assert(offsetof(observer_view_t, qkick_phase) > offsetof(observer_view_t, axis_ok), "view.qkick_phase");
_Static_assert(offsetof(observer_view_t, qkick_seed) > offsetof(observer_view_t, qkick_phase), "view.qkick_seed");
_Static_assert(offsetof(observer_view_t, qkick_dth) > offsetof(observer_view_t, qkick_seed), "view.qkick_dth");

static void hfi_ops_init(void)
{
    hfi_sqwave_init();
}

static void hfi_ops_reset(void)
{
    hfi_sqwave_reset();
}

static float hfi_ops_get_omega(void)
{
    return hfi_sqwave_get_pll_int_el();
}

static uint8_t hfi_ops_is_converged(void)
{
    return (hfi_sqwave_get_lock() == HFI_LOCK_LOCKED) ? 1u : 0u;
}

static float hfi_ops_get_theta(void)
{
    float theta_park = obs_get_theta();

    if (hfi_pub_smo_active() != 0u) {
        theta_park = hfi_pub_theta();
    }
    return theta_park;
}

static const observer_ops_t s_hfi_ops = {
    hfi_ops_init,
    hfi_ops_reset,
    obs_pre_park,
    hfi_ops_get_theta,
    obs_post_park,
    hfi_ops_get_omega,
    hfi_ops_is_converged,
};

static const observer_inj_ops_t s_hfi_inj = {
    obs_override_voltage,
    obs_get_inj,
    obs_get_inj_ab,
};

static const observer_src_slot_t s_lo_src = {
    observer_get_theta_hat,
    hfi_ops_get_omega,
    observer_speed_run_active,
};

static const observer_ops_t *s_obs;
static const observer_inj_ops_t *s_inj;
static emf_pll_t *s_bound_emf_pll;

void observer_composite_init(void)
{
    s_obs = &s_hfi_ops;
    s_inj = &s_hfi_inj;
    s_obs->init();
}

void observer_bringup(void *emf_pll)
{
    observer_composite_init();
    observer_bind_emf_pll(emf_pll);
}

void observer_bind_emf_pll(void *pll)
{
    s_bound_emf_pll = (emf_pll_t *)pll;
    obs_angle_bind_pll(pll);
}

float observer_emf_theta_hat(void)
{
    return (s_bound_emf_pll != 0) ? s_bound_emf_pll->theta_hat : 0.0f;
}

float observer_emf_omega_el(void)
{
    return (s_bound_emf_pll != 0) ? s_bound_emf_pll->omega_el : 0.0f;
}

float observer_emf_theta_err(void)
{
    return (s_bound_emf_pll != 0) ? s_bound_emf_pll->theta_err : 0.0f;
}

float observer_emf_last_pd(void)
{
    return (s_bound_emf_pll != 0) ? s_bound_emf_pll->last_pd : 0.0f;
}

void observer_emf_reset(void)
{
    if (s_bound_emf_pll != 0) {
        emf_pll_reset(s_bound_emf_pll);
    }
}

void observer_emf_update(float e_alpha, float e_beta, float theta_enc, float dt)
{
    if (s_bound_emf_pll != 0) {
        emf_pll_update(s_bound_emf_pll, e_alpha, e_beta, theta_enc, dt);
    }
}

const observer_src_slot_t *observer_lo_src(void)
{
    return &s_lo_src;
}

void observer_lo_set_inj_scale(float scale)
{
    hfi_sqwave_set_inj_scale(scale);
}

void observer_lo_set_iq_auth_hold(uint8_t hold)
{
    hfi_sqwave_set_iq_auth_hold(hold);
}

void observer_lo_seed_hat(float theta_el, float omega_el)
{
    hfi_sqwave_seed_hat(theta_el, omega_el);
}

const observer_ops_t *observer_ops(void)
{
    return s_obs;
}

const observer_inj_ops_t *observer_inj_ops(void)
{
    return s_inj;
}

void observer_telem_publish(void)
{
    hfi_sqwave_telem_publish();
}

void observer_pub_step(float theta_smo, float omega_el, float dth)
{
    hfi_pub_step(theta_smo, omega_el, dth);
}

float observer_pub_ss(void)
{
    return hfi_pub_ss();
}

float observer_pub_smo_rpm(void)
{
    return hfi_pub_smo_rpm();
}

void observer_smo_w_ma_reset(void)
{
    hfi_smo_w_ma_reset();
}

float observer_smo_w_ma_step(float rpm)
{
    return hfi_smo_w_ma_step(rpm);
}

uint8_t observer_get_stage(void)
{
    return (uint8_t)hfi_sqwave_get_stage();
}

uint8_t observer_get_lock(void)
{
    return (uint8_t)hfi_sqwave_get_lock();
}

float observer_get_pll_int_el(void)
{
    return hfi_sqwave_get_pll_int_el();
}

float observer_get_omega_el(void)
{
    return hfi_sqwave_get_omega_el();
}

uint8_t observer_consume_pi_reset(void)
{
    return hfi_sqwave_consume_pi_reset();
}

uint8_t observer_speed_run_active(void)
{
    return hfi_sqwave_speed_run_active();
}

float observer_get_speed_ref_rpm(void)
{
    return hfi_sqwave_get_speed_ref_rpm();
}

float observer_get_id_ref(void)
{
    return hfi_sqwave_get_id_ref();
}

float observer_get_iq_ref(void)
{
    return hfi_sqwave_get_iq_ref();
}

uint8_t observer_if_leave_active(void)
{
    return hfi_sqwave_if_leave_active();
}

void observer_set_omega_ff_el(float omega_el_rad_s)
{
    hfi_sqwave_set_omega_ff_el(omega_el_rad_s);
}

float observer_get_iq_auth_abs(void)
{
    return hfi_sqwave_get_iq_auth_abs();
}

uint8_t observer_id_pi_bypass(void)
{
    return hfi_sqwave_id_pi_bypass();
}

float observer_id_pi_soft_scale(void)
{
    return hfi_sqwave_id_pi_soft_scale();
}

float observer_get_theta_hat(void)
{
    return hfi_sqwave_get_theta_hat();
}

void observer_read_view(observer_view_t *v)
{
    hfi_telem_snap_t snap;

    if (v == 0) {
        return;
    }
    hfi_sqwave_telem_harvest(&snap);
    v->stage = (uint8_t)hfi_sqwave_get_stage();
    v->lock = (uint8_t)hfi_sqwave_get_lock();
    v->theta_cmd = hfi_sqwave_get_theta_cmd();
    v->theta_hat = hfi_sqwave_get_theta_hat();
    v->theta_err = hfi_sqwave_get_theta_err();
    v->eps = snap.eps;
    v->di_q = snap.di_q;
    v->di_d = snap.di_d;
    v->x_raw = snap.x_raw;
    v->y_raw = snap.y_raw;
    v->vh_sign = snap.vh_sign;
    v->eps_dead = hfi_sqwave_get_eps_dead();
    v->pll_int_el = snap.pll_int_el;
    v->omega_el = hfi_sqwave_get_omega_el();
    v->omega_trim_el = hfi_sqwave_get_omega_trim_el();
    v->ipd_phase = hfi_sqwave_get_ipd_phase();
    v->ipd_pulse_ud = hfi_sqwave_get_ipd_pulse_ud();
    v->eps_d = hfi_sqwave_get_eps_d();
    v->qkick_verdict = snap.qkick_verdict;
    v->axis_flip_n = hfi_sqwave_get_axis_flip_n();
    v->axis_ok = hfi_sqwave_axis_ok();
    v->qkick_phase = hfi_sqwave_get_qkick_phase();
    v->qkick_seed = hfi_sqwave_get_qkick_seed();
    v->qkick_dth = snap.qkick_dth;
}

uint8_t observer_take_polarity_flip(void)
{
    return hfi_sqwave_take_polarity_flip();
}

#else /* !M1_HFI_ENABLE */

#include "observer/obs_angle.h"
#include "observer/emf_pll.h"

static void obs_stub_init(void) {}
static void obs_stub_reset(void) {}
static void obs_stub_pre_park(float theta_enc, float dt)
{
    (void)theta_enc;
    (void)dt;
}
static float obs_stub_get_theta(void) { return 0.0f; }
static void obs_stub_post_park(float id, float iq, float ia, float ib)
{
    (void)id;
    (void)iq;
    (void)ia;
    (void)ib;
}
static float obs_stub_get_omega(void) { return 0.0f; }
static uint8_t obs_stub_is_converged(void) { return 0u; }

static const observer_ops_t s_stub_ops = {
    obs_stub_init,
    obs_stub_reset,
    obs_stub_pre_park,
    obs_stub_get_theta,
    obs_stub_post_park,
    obs_stub_get_omega,
    obs_stub_is_converged,
};

static const observer_ops_t *s_obs = &s_stub_ops;
static const observer_inj_ops_t *s_inj;
static emf_pll_t *s_bound_emf_pll;

static const observer_src_slot_t s_lo_src = {
    observer_get_theta_hat,
    obs_stub_get_omega,
    observer_speed_run_active,
};

void observer_composite_init(void)
{
    s_obs = &s_stub_ops;
    s_inj = 0;
}

void observer_bringup(void *emf_pll)
{
    observer_composite_init();
    observer_bind_emf_pll(emf_pll);
}

void observer_bind_emf_pll(void *pll)
{
    s_bound_emf_pll = (emf_pll_t *)pll;
    obs_angle_bind_pll(pll);
}

float observer_emf_theta_hat(void)
{
    return (s_bound_emf_pll != 0) ? s_bound_emf_pll->theta_hat : 0.0f;
}

float observer_emf_omega_el(void)
{
    return (s_bound_emf_pll != 0) ? s_bound_emf_pll->omega_el : 0.0f;
}

float observer_emf_theta_err(void)
{
    return (s_bound_emf_pll != 0) ? s_bound_emf_pll->theta_err : 0.0f;
}

float observer_emf_last_pd(void)
{
    return (s_bound_emf_pll != 0) ? s_bound_emf_pll->last_pd : 0.0f;
}

void observer_emf_reset(void)
{
    if (s_bound_emf_pll != 0) {
        emf_pll_reset(s_bound_emf_pll);
    }
}

void observer_emf_update(float e_alpha, float e_beta, float theta_enc, float dt)
{
    if (s_bound_emf_pll != 0) {
        emf_pll_update(s_bound_emf_pll, e_alpha, e_beta, theta_enc, dt);
    }
}

const observer_src_slot_t *observer_lo_src(void)
{
    return &s_lo_src;
}

void observer_lo_set_inj_scale(float scale)
{
    (void)scale;
}

void observer_lo_set_iq_auth_hold(uint8_t hold)
{
    (void)hold;
}

void observer_lo_seed_hat(float theta_el, float omega_el)
{
    (void)theta_el;
    (void)omega_el;
}

const observer_ops_t *observer_ops(void)
{
    return s_obs;
}

const observer_inj_ops_t *observer_inj_ops(void)
{
    return s_inj;
}

void observer_telem_publish(void) {}
void observer_pub_step(float theta_smo, float omega_el, float dth)
{
    (void)theta_smo;
    (void)omega_el;
    (void)dth;
}
float observer_pub_ss(void) { return 0.0f; }
float observer_pub_smo_rpm(void) { return 0.0f; }
void observer_smo_w_ma_reset(void) {}
float observer_smo_w_ma_step(float rpm)
{
    (void)rpm;
    return 0.0f;
}
uint8_t observer_get_stage(void) { return OBS_STAGE_IDLE; }
uint8_t observer_get_lock(void) { return OBS_LOCK_CAPTURE; }
float observer_get_pll_int_el(void) { return 0.0f; }
float observer_get_omega_el(void) { return 0.0f; }
uint8_t observer_consume_pi_reset(void) { return 0u; }
uint8_t observer_speed_run_active(void) { return 0u; }
float observer_get_speed_ref_rpm(void) { return 0.0f; }
float observer_get_id_ref(void) { return 0.0f; }
float observer_get_iq_ref(void) { return 0.0f; }
uint8_t observer_if_leave_active(void) { return 0u; }
void observer_set_omega_ff_el(float omega_el_rad_s) { (void)omega_el_rad_s; }
float observer_get_iq_auth_abs(void) { return 0.0f; }
uint8_t observer_id_pi_bypass(void) { return 0u; }
float observer_id_pi_soft_scale(void) { return 1.0f; }
float observer_get_theta_hat(void) { return 0.0f; }
void observer_read_view(observer_view_t *v)
{
    if (v == 0) {
        return;
    }
    v->stage = OBS_STAGE_IDLE;
    v->lock = OBS_LOCK_CAPTURE;
    v->theta_cmd = 0.0f;
    v->theta_hat = 0.0f;
    v->theta_err = 0.0f;
    v->eps = 0.0f;
    v->di_q = 0.0f;
    v->di_d = 0.0f;
    v->x_raw = 0.0f;
    v->y_raw = 0.0f;
    v->vh_sign = 0.0f;
    v->eps_dead = 0.0f;
    v->pll_int_el = 0.0f;
    v->omega_el = 0.0f;
    v->omega_trim_el = 0.0f;
    v->ipd_phase = 0u;
    v->ipd_pulse_ud = 0.0f;
    v->eps_d = 0.0f;
    v->qkick_verdict = 0.0f;
    v->axis_flip_n = 0.0f;
    v->axis_ok = 0u;
    v->qkick_phase = 0u;
    v->qkick_seed = 0.0f;
    v->qkick_dth = 0.0f;
}
uint8_t observer_take_polarity_flip(void) { return 0u; }

#endif /* M1_HFI_ENABLE */
