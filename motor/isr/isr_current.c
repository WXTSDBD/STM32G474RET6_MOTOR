/**
 * @file isr_current.c
 * @date 2026-10-29
 * @brief Park、电流 PI、Ud/Uq 合成、前馈、观测器更新。
 */
#include "isr/isr_frame.h"
#include "isr/isr_state.h"

#include <stddef.h>

#include "dbg_monitor.h"
#include "deadband_flow.h"
#include "deadband_module.h"
#include "foc_pi.h"
#include "foc_svpwm.h"
#include "motor_current_exp_hooks.h"
#include "motor_foc_loop.h"
#include "motor_open_sweep.h"
#include "motor_params_m1.h"
#include "motor_startup.h"
#include "motor_trig.h"
#include "core/motor_route.h"

#include "experiment/exp_runner.h"
#include "experiment/exp_act_basic.h"
#include "observer/observer_composite.h"

/**
 * @brief 电流环到观测器：Park→PI→电压合成→ff→SMO/EMF。
 * @param f 帧；读写 id/iq/ud/uq/sin/cos 与 DWT 观测段打点。
 */
void isr_current_run(isr_frame_t *f)
{
    motor_context_t *ctx;

    if ((f == NULL) || (f->ctx == NULL)) {
        return;
    }
    ctx = f->ctx;
    (void)ctx;

    motor_trig_sincos(f->theta_park, &f->cos_el, &f->sin_el);
    Park_Transform_sc(f->i_alpha, f->i_beta, f->sin_el, f->cos_el, &f->id, &f->iq);
    f->ctx->id = f->id;
    f->ctx->iq = f->iq;

    /* 解调只走 angle_ops->post_park；有感表该项为 NULL。标定锁轴跳过。 */
    if (deadband_flow_id_cal_active() == 0u) {
        const angle_ops_t *ang = motor_angle_active();

        if ((ang != NULL) && (ang->post_park != NULL)) {
            ang->post_park(&g_isr_m1_ctx, f->id, f->iq, f->i_alpha, f->i_beta);
        }
    }
    if (M1_HFI_ENABLE && M1_HFI_IQ_AUTH_ENABLE) {
        /*
         * 注入仍开时扫权威限幅。交接后 inj_off：天花板已钉 HI（=速度环满幅），
         * 解调也不再推 auth，PI 限幅保持上一拍即可。
         */
        if (observer_hfi_inj_off() == 0u) {
            if (observer_speed_run_active() != 0u) {
                const float lim = observer_get_iq_auth_abs();

                if (f->ctx->iq_ref > lim) {
                    f->ctx->iq_ref = lim;
                } else if (f->ctx->iq_ref < -lim) {
                    f->ctx->iq_ref = -lim;
                }
                f->ctx->pi_speed.out_max = lim;
                f->ctx->pi_speed.out_min = -lim;
                f->ctx->pi_speed.int_max = lim;
                f->ctx->pi_speed.int_min = -lim;
                if (f->ctx->pi_speed.integrator > lim) {
                    f->ctx->pi_speed.integrator = lim;
                } else if (f->ctx->pi_speed.integrator < -lim) {
                    f->ctx->pi_speed.integrator = -lim;
                }
            } else {
                f->ctx->pi_speed.out_max = M1_SPEED_PI_OUT_MAX;
                f->ctx->pi_speed.out_min = M1_SPEED_PI_OUT_MIN;
                f->ctx->pi_speed.int_max = M1_SPEED_PI_INT_MAX;
                f->ctx->pi_speed.int_min = M1_SPEED_PI_INT_MIN;
            }
        }
    }

    if (M1_HFI_ENABLE) {
        /*
         * 无扰：f->id* = (1-soft)·f->id → 误差 = -soft·f->id；
         * soft=0 时误差为 0；soft→1 → f->id*→0。须在 foc_loop 之前。
         * 标定链（Pass0/Rs/L）与 HFI 互斥：表在跑就别改 id_ref。
         */
        if ((deadband_flow_id_cal_active() == 0u) &&
            (observer_id_pi_bypass() == 0u)) {
            const float soft = observer_id_pi_soft_scale();

            if (soft < 1.0f) {
                f->ctx->id_ref = (1.0f - soft) * f->id;
            } else {
                f->ctx->id_ref = 0.0f;
            }
        }
    }

    motor_foc_loop_dbg_id_ref(ctx);

    if (f->ctx->mode == M1_CTRL_CURRENT_LOOP) {
        float id_for_pi = f->id;

        motor_startup_finish_tick(ctx, f->iq, &f->startup);
        if (M1_EXP_FRAMEWORK_ENABLE) {
            if ((exp_runner_is_armed() != 0u) &&
                (exp_runner_active_id() == EXP_ID_UD_CAL) &&
                (exp_act_theta_is_active() != 0u)) {
                f->ctx->id_ref = exp_act_theta_get_id_a();
                f->ctx->iq_ref = 0.0f;
                f->ctx->mode = M1_CTRL_CURRENT_LOOP;
                f->startup.iq_ref = 0.0f;
                f->startup.use_fixed_uq = 0u;
            }
            if ((exp_runner_is_armed() != 0u) &&
                (exp_runner_active_id() == EXP_ID_REV141)) {
                f->ctx->mode = M1_CTRL_CURRENT_LOOP;
            }
        }
        motor_foc_loop_tick(ctx, id_for_pi, f->iq, &f->startup, f->theta_enc_park);
    }

    if (f->ctx->mode == M1_CTRL_OBSERVE_ONLY || f->ctx->mode == M1_CTRL_OPEN_LOOP) {
        motor_open_sweep_tick(ctx);
    }

    switch (f->ctx->mode) {
    case M1_CTRL_CURRENT_LOOP:
        if (motor_current_exp_try_ld_lq_udq(ctx, f->id, f->iq, &f->ud_out, &f->uq_out) !=
            0u) {
            break;
        }
        if (f->startup.use_fixed_uq) {
            f->ud_out = 0.0f;
            f->uq_out = f->startup.uq_out;
        } else {
            f->ud_out = f->ctx->ud_pi + deadband_service_ud_inject(f->id);
            f->uq_out = f->ctx->uq_pi;
            if (M1_HFI_ENABLE) {
                /* 标定链：HFI 不掺 Ud；其余旁路/soft 与 141 无感一致。 */
                if (deadband_flow_id_cal_active() != 0u) {
                    dbg.foc_ud_pi = f->ctx->ud_pi;
                } else if (observer_id_pi_bypass() != 0u) {
                    foc_pi_reset(&f->ctx->pi_id);
                    f->ctx->ud_pi = 0.0f;
                    dbg.foc_ud_pi = 0.0f;
                    f->ud_out = deadband_service_ud_inject(f->id);
                } else {
                    float soft = observer_id_pi_soft_scale();
                    float ud_db = deadband_service_ud_inject(f->id);

                    f->ud_out = ud_db + soft * f->ctx->ud_pi;
                    dbg.foc_ud_pi = soft * f->ctx->ud_pi;
                }
            }
        }
        if (M1_HFI_ENABLE) {
            /*
             * 标定链（Rs/L/Pass0）与 HFI 互斥：IDLE 的 override(0,0) 会盖住 Rs 斜坡
             * 并每拍清 PI（表现为前几秒无电流、随后 id_ref 已爬高而突跳）。
             */
            if (deadband_flow_id_cal_active() == 0u) {
                float ud_ov;
                float uq_ov;
                float ud_inj;
                float uq_inj;

                if ((g_isr_m1_inj != 0) &&
                    (g_isr_m1_inj->override_voltage(&ud_ov, &uq_ov) != 0u)) {
                    f->ud_out = ud_ov;
                    f->uq_out = uq_ov;
                    if ((ud_ov == 0.0f) && (uq_ov == 0.0f)) {
                        foc_pi_reset(&f->ctx->pi_id);
                        foc_pi_reset(&f->ctx->pi_iq);
                        f->ctx->ud_pi = 0.0f;
                        f->ctx->uq_pi = 0.0f;
                        dbg.foc_ud_pi = 0.0f;
                        dbg.foc_uq_pi = 0.0f;
                    }
                } else if ((g_isr_m1_inj != 0) &&
                           (observer_hfi_inj_off() == 0u) &&
                           (((motor_angle_active() != NULL) &&
                             (motor_angle_sensorless() != 0u)) ||
                            ((motor_angle_active() == NULL) &&
                             (observer_sensed_fb_active() == 0u)))) {
                    g_isr_m1_inj->get_inj(&ud_inj, &uq_inj);
                    f->ud_out += ud_inj;
                    f->uq_out += uq_inj;
                    f->ua_hfi = ud_inj * f->cos_el - uq_inj * f->sin_el;
                    f->ub_hfi = ud_inj * f->sin_el + uq_inj * f->cos_el;
                    {
                        float ua_inj;
                        float ub_inj;

                        g_isr_m1_inj->get_inj_ab(&ua_inj, &ub_inj);
                        f->ud_out += ua_inj * f->cos_el + ub_inj * f->sin_el;
                        f->uq_out += -ua_inj * f->sin_el + ub_inj * f->cos_el;
                        f->ua_hfi += ua_inj;
                        f->ub_hfi += ub_inj;
                    }
                }
            }
        }
        break;
    case M1_CTRL_OBSERVE_ONLY:
        f->ud_out = 0.0f;
        f->uq_out = f->ctx->uq_open;
        break;
    case M1_CTRL_OPEN_LOOP:
        if (motor_current_exp_try_open_udq(ctx, &f->ud_out, &f->uq_out) != 0u) {
            break;
        }
        f->ud_out = 0.0f;
        f->uq_out = f->ctx->uq_open;
        break;
    default:
        f->ud_out = 0.0f;
        f->uq_out = 0.0f;
        break;
    }

    {
        const ff_ops_t *ff = motor_ff_active();

        if ((ff != NULL) && (ff->apply != NULL)) {
            ff->apply(ctx, &f->ud_out, &f->uq_out, f->id, f->iq);
        }
    }
    dbg.foc_uq_out = f->uq_out;
    dbg.foc_ud_out = f->ud_out;
}
