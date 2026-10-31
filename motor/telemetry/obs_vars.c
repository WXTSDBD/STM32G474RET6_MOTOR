/**
 * @file obs_vars.c
 * @date 2026-10-31
 * @brief A 类观测登记：dbg 直连 + 每拍镜像（供 OPT/签收预设）。
 */
#include "telemetry/obs_reg.h"

#include "app_uart_dma_debug.h"
#include "dbg_monitor.h"
#include "motor_current.h"
#include "motor_math.h"
#include "motor_params_m1.h"

#if M1_HFI_ENABLE
#include "observer/observer_composite.h"
#endif
#include "app_axis_jeoc.h"

static float obs_isr_delta_f;
static float obs_seq_f;
static float obs_mark_f;
static float obs_duty_dev_f;
static float obs_theta_park_err_f;
static float obs_status_f;
static float obs_omega_fb_f;
static float obs_theta_mech_f;
static float obs_theta_ref_f;

float g_obs_cli_marker;
float g_obs_rx_bytes;

OBSERVE_AS(foc_ia, "ia", dbg.foc_ia);
OBSERVE_AS(foc_ib, "ib", dbg.foc_ib);
OBSERVE_AS(foc_ic, "ic", dbg.foc_ic);
OBSERVE_AS(foc_id, "id", dbg.foc_id);
OBSERVE_AS(foc_iq, "iq", dbg.foc_iq);
OBSERVE_AS(foc_theta, "theta_el", dbg.foc_theta_el);
OBSERVE_AS(foc_ud, "ud", dbg.foc_ud_out);
OBSERVE_AS(foc_uq, "uq", dbg.foc_uq_out);
OBSERVE_AS(foc_ud_pi, "ud_pi", dbg.foc_ud_pi);
OBSERVE_AS(id_ref, "id_ref", dbg.foc_id_ref);
OBSERVE_AS(iq_ref, "iq_ref", dbg.foc_iq_ref);
OBSERVE_AS(omega_pll, "omega_pll", dbg.pll_omega_mech_rpm);
OBSERVE_AS(omega_ref, "omega_ref", dbg.outer_omega_ref);
OBSERVE_AS(theta_err, "theta_err", dbg.outer_theta_err_rad);
OBSERVE_AS(theta_mech, "theta_mech", obs_theta_mech_f);
OBSERVE_AS(theta_ref, "theta_ref", obs_theta_ref_f);
OBSERVE_AS(enc_raw, "enc_raw", dbg.enc_raw);
OBSERVE_AS(duty_dev, "duty_dev", obs_duty_dev_f);
OBSERVE_AS(mark, "mark", obs_mark_f);
OBSERVE_AS(theta_park_err, "theta_park_err", obs_theta_park_err_f);
OBSERVE_AS(status, "status", obs_status_f);
OBSERVE_AS(omega_fb, "omega_fb", obs_omega_fb_f);
OBSERVE_AS(isr_delta, "isr_delta", obs_isr_delta_f);
OBSERVE_AS(seq_mirror, "seq", obs_seq_f);
OBSERVE_AS(cli_marker, "marker", g_obs_cli_marker);
OBSERVE_AS(rx_bytes, "rx_bytes", g_obs_rx_bytes);

/**
 * @brief 每拍刷新派生量（遥测路径调用，不进 FOC 热路径核心）。
 */
void obs_vars_tick_mirrors(void)
{
    uint32_t st = 0u;

    obs_isr_delta_f = (float)g_telem_dbg.isr_delta;
    obs_seq_f = (float)obs_session_seq();
    obs_mark_f = (float)dbg.open_seq_phase;
    obs_duty_dev_f = dbg.foc_svpwm_duty_dev;
    obs_theta_mech_f = dbg.outer_theta_mech_rad;
    obs_theta_ref_f = dbg.outer_theta_ref_rad;
    obs_theta_park_err_f =
        motor_wrap_pi(dbg.foc_theta_el - dbg.enc_theta_el);
    obs_omega_fb_f = motor_current_get_pll_omega_mech_rpm();

    if (dbg.hfi_stage > 0.0f) {
        st |= 0x01u;
    }
#if M1_HFI_ENABLE
    if (observer_sensed_fb_active() != 0u) {
        st |= 0x02u;
    }
#endif
    st |= (app_axis_jeoc_diag_bits() & 0x0Cu);
    obs_status_f = (float)st;
}
