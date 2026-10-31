/**
 * @file isr_startup.c
 * @date 2026-10-29
 * @brief 启动/I-f/angle_ops 选 Park 角与运动旁路写 ref。
 */
#include "isr/isr_frame.h"
#include "isr/isr_state.h"

#include <stddef.h>

#include "dbg_monitor.h"
#include "deadband_flow.h"
#include "deadband_id_cal.h"
#include "exp_mark.h"
#include "foc_pi.h"
#include "motor_outer_loop.h"
#include "motor_params_m1.h"
#include "motor_startup.h"
#include "motor_if.h"
#include "core/motor_route.h"

#include "experiment/exp_runner.h"
#include "experiment/exp_act_basic.h"
#include "observer/observer_composite.h"

static uint8_t exp_owns_drive(void)
{
    return (exp_runner_is_armed() != 0u) ? 1u : 0u;
}

/**
 * @brief 选本拍 Park 角，并按观测/实验写 id/iq/ω 指令。
 * @param f 帧；读 theta_enc_park，写 theta_park / startup / 部分 ref。
 */
void isr_startup_run(isr_frame_t *f)
{
    motor_context_t *ctx;

    if ((f == NULL) || (f->ctx == NULL)) {
        return;
    }
    ctx = f->ctx;
    (void)ctx;

    if (f->ctx->mode == M1_CTRL_CURRENT_LOOP) {
        f->startup = motor_startup_tick(ctx, f->theta_enc_park);
        if (M1_ID_LOCK_CAL_SWEEP) {
            if (deadband_id_cal_use_fix_theta() ||
                (deadband_flow_id_cal_active() &&
                 !deadband_id_cal_in_iq_probe())) {
                f->theta_park = deadband_id_cal_target_theta();
            } else {
                f->theta_park = f->theta_enc_park;
            }
        } else if (!M1_IDENT_ENABLE) {
            f->theta_park = f->startup.theta_park;
        }
        dbg.startup_state = (uint8_t)f->startup.state;
        dbg.startup_omega_mech_rpm = f->startup.omega_mech_rpm;
        dbg.if_omega_cmd_rpm = 0.0f;
        dbg.if_theta_err_rad = 0.0f;
    } else {
        f->theta_park = f->theta_enc_park;
        dbg.startup_state = (uint8_t)M1_STARTUP_CLOSED;
        dbg.startup_omega_mech_rpm = 0.0f;
        dbg.if_omega_cmd_rpm = 0.0f;
        dbg.if_theta_err_rad = 0.0f;
        f->startup.iq_ref = 0.0f;
        f->startup.state = M1_STARTUP_CLOSED;
        f->startup.use_fixed_uq = 0u;
        f->startup.uq_out = 0.0f;
        f->startup.pi_bumpless = 0u;
        f->startup.uq_prev = 0.0f;
        f->startup.ud_prev = 0.0f;
    }
    /*
     * Park 角只走 angle_ops（route 必挂表）。无表时用编码器角（未绑 route 的过渡）。
     * 标定锁轴：上方已写死 f->theta_park，此处跳过以免盖掉 FIX_THETA。
     */
    if (deadband_flow_id_cal_active() == 0u) {
        const angle_ops_t *ang = motor_angle_active();

        if (ang != NULL) {
            motor_angle_note_enc(f->theta_enc_park);
            if (ang->update != NULL) {
                ang->update(&g_isr_m1_ctx, M1_CTRL_TS_S);
            }
            f->theta_park = ang->get_theta(&g_isr_m1_ctx);
            if (M1_HFI_ENABLE && M1_HFI_MOTION_BYPASS_ENABLE) {
                if ((ang->post_park != NULL) &&
                    (observer_get_stage() == OBS_STAGE_RUN)) {
                    g_isr_hfi_recon_theta = f->theta_park;
                    g_isr_hfi_recon_theta_ok = 1u;
                }
            }
            if (M1_HFI_ENABLE && M1_HFI_QKICK_AFTER_LOCK_ENABLE) {
                if ((ang->post_park != NULL) &&
                    (observer_consume_pi_reset() != 0u)) {
                    foc_pi_reset(&f->ctx->pi_iq);
                    f->ctx->uq_pi = 0.0f;
                }
            }
        } else {
            f->theta_park = f->theta_enc_park;
        }
    }
    if (M1_HFI_ENABLE) {
        /*
         * 进有感段（REV141 SENSED_PAIRS）第一拍：电流环 PI 与 Ud/Uq 全部清零。
         */
        if (observer_consume_sens_pi_clear() != 0u) {
            foc_pi_reset(&f->ctx->pi_id);
            foc_pi_reset(&f->ctx->pi_iq);
            f->ctx->ud_pi = 0.0f;
            f->ctx->uq_pi = 0.0f;
        }
        if (M1_HFI_MOTION_BYPASS_ENABLE && M1_SPEED_LOOP_ENABLE) {
            if (observer_speed_run_active() != 0u) {
                const float rpm_cmd = observer_get_speed_ref_rpm();

                if (exp_owns_drive() == 0u) {
                    f->ctx->omega_ref = rpm_cmd;
                }
                if (f->ctx->outer_mode != M1_OUTER_SPEED) {
                    f->ctx->omega_ref = 0.0f;
                    motor_outer_set_mode(ctx, M1_OUTER_SPEED, 0.0f, 0.0f);
                    if (exp_owns_drive() == 0u) {
                        f->ctx->omega_ref = rpm_cmd;
                    }
                }
            } else {
                if (exp_owns_drive() == 0u) {
                    f->ctx->id_ref = observer_get_id_ref();
                    f->ctx->iq_ref = observer_get_iq_ref();
                    f->ctx->omega_ref = observer_get_speed_ref_rpm();
                }
                if (f->ctx->outer_mode != M1_OUTER_DISABLED) {
                    motor_outer_set_mode(ctx, M1_OUTER_DISABLED, 0.0f, 0.0f);
                }
            }
            observer_set_omega_ff_el(0.0f);
        }
    }
    dbg.foc_theta_el = f->theta_park;

    if (M1_EXP_FRAMEWORK_ENABLE) {
        if (exp_runner_is_armed() == 0u) {
            exp_act_theta_clear();
        } else if ((exp_runner_active_id() == EXP_ID_UD_CAL) &&
                   (exp_act_theta_is_active() != 0u)) {
            f->theta_park = exp_act_theta_get_rad();
            dbg.foc_theta_el = f->theta_park;
        } else if ((exp_runner_active_id() == EXP_ID_REV141) &&
                   (exp_act_angle_src_is_active() != 0u) &&
                   (exp_act_angle_src() != 0u)) {
            f->theta_park = f->theta_enc_park;
            dbg.foc_theta_el = f->theta_park;
        }
    }
}
