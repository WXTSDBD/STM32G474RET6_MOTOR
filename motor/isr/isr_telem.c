/**
 * @file isr_telem.c
 * @date 2026-10-29
 * @brief 观测器更新 + 拍末遥测 / DWT 记账（须在 SVPWM 前跑完 SMO/EMF）。
 */
#include "isr/isr_frame.h"
#include "isr/isr_state.h"

#include "app_uart_dma_debug.h"
#include "dbg_monitor.h"
#include "exp_mark.h"
#include "motor_outer_loop.h"
#include "motor_params_m1.h"
#include "core/motor_route.h"

#include "stm32g474xx.h"

#if M1_EMF_SMO_ENABLE
#include "observer/emf_smo.h"
#endif
#if M1_EMF_PLL_ENABLE
#include "observer/emf_pll.h"
#endif
#if M1_HFI_ENABLE
#include "observer/observer_composite.h"
#endif

/**
 * @brief harvest + SMO/EMF/pub，并打 obs_delta。
 * @param f 帧；须已填 ud/uq/sin/cos/ua_hfi 与 theta_enc_park。
 * @note 在 isr_modulate_run 之前调用。
 */
void isr_obs_run(isr_frame_t *f)
{
    if (f == NULL) {
        return;
    }

#if M1_HFI_ENABLE
    /* 瘦路径：只 harvest + OPT12 需要的 hfi_stage（勿再满拷 read_view） */
    observer_telem_harvest();
    dbg.hfi_stage =
        (float)observer_get_stage() + observer_get_eps_dead();
#if (M1_OUTER_THETA_FB_SRC == 2) && M1_OUTER_NEST_ENABLE && \
    (M1_OUTER_EXPT == M1_OUTER_EXPT_SIGNOFF)
    if ((motor_outer_signoff_is_armed() == 0u) &&
        (dbg.open_seq_phase != EXP_MARK_SIGN_DONE) &&
        (dbg.open_seq_phase != EXP_MARK_SIGN_GUARD)) {
        dbg.open_seq_phase = (uint8_t)observer_get_stage();
    }
#endif
#endif

    f->t_pre_obs = *(volatile uint32_t *)&DWT->CYCCNT;
#if M1_EMF_VEQ_ENABLE || M1_EMF_SMO_ENABLE || M1_EMF_PLL_ENABLE
#if M1_HFI_ENABLE
    if (((motor_angle_active() != NULL) && (motor_angle_sensorless() == 0u)) ||
        ((motor_angle_active() == NULL) && (observer_sensed_fb_active() != 0u))) {
        /* 有感段：观测器热路径空转。 */
    } else
#endif
    {
        float omega_mech_rpm = 0.0f;
        float u_alpha;
        float u_beta;
        float theta_obs_ref = f->theta_enc_park;
        uint8_t smo_hold = 0u;

#if M1_PLL_ENABLE
        omega_mech_rpm = g_isr_speed_fb_rpm;
#endif
#if M1_HFI_ENABLE && M1_HFI_MOTION_BYPASS_ENABLE && M1_EMF_PLL_ENABLE && \
    (M1_EMF_SMO_ENABLE || M1_EMF_VEQ_ENABLE)
        theta_obs_ref = observer_get_theta_hat();
        (void)observer_take_polarity_flip();
#endif
        if (smo_hold == 0u) {
            u_alpha = f->ud_out * f->cos_el - f->uq_out * f->sin_el;
            u_beta = f->ud_out * f->sin_el + f->uq_out * f->cos_el;
            u_alpha -= f->ua_hfi;
            u_beta -= f->ub_hfi;

#if M1_EMF_SMO_ENABLE
            (void)emf_smo_lpf_sched_update(omega_mech_rpm);
            emf_smo_update(&g_isr_emf_smo, f->i_alpha, f->i_beta, u_alpha, u_beta,
                           theta_obs_ref, omega_mech_rpm);
#endif
#if M1_EMF_PLL_ENABLE
#if M1_EMF_PLL_USE_SMO
            observer_emf_update(g_isr_emf_smo.e_alpha, g_isr_emf_smo.e_beta,
                                theta_obs_ref, M1_CTRL_TS_S);
#else
            observer_emf_update(s_emf_veq.e_alpha, s_emf_veq.e_beta,
                                theta_obs_ref, M1_CTRL_TS_S);
#endif
#if M1_HFI_ENABLE && M1_HFI_MOTION_BYPASS_ENABLE && M1_EMF_PLL_ENABLE
            {
                const float emf_th = observer_emf_theta_hat();
                const float emf_w = observer_emf_omega_el();
                float dth = emf_th - theta_obs_ref;

                while (dth > 3.14159265f) {
                    dth -= 6.2831853f;
                }
                while (dth < -3.14159265f) {
                    dth += 6.2831853f;
                }
                observer_pub_step(emf_th, emf_w, dth);
            }
#endif
#endif
        }
    }
#endif
    f->t_post_obs = *(volatile uint32_t *)&DWT->CYCCNT;
    g_telem_dbg.obs_delta = f->t_post_obs - f->t_pre_obs;
}

/**
 * @brief 拍末遥测与 ISR 周期统计。
 * @param f 帧；须已填 isr_t0 / foc_t0 / t_pre_obs / t_post_obs / t_post_svpwm。
 */
void isr_telem_run(isr_frame_t *f)
{
    volatile uint32_t isr_t1;

    if (f == NULL) {
        return;
    }

#if M1_HFI_ENABLE
    observer_telem_publish();
#endif
    telem_bringup_tick();

    isr_t1 = *(volatile uint32_t *)&DWT->CYCCNT;
    g_telem_dbg.isr_t0 = f->isr_t0;
    g_telem_dbg.isr_t1 = isr_t1;
    g_telem_dbg.cyccnt_end = isr_t1;
    g_telem_dbg.isr_delta = isr_t1 - f->isr_t0;
    g_telem_dbg.foc_delta =
        (f->t_pre_obs - f->foc_t0) + (f->t_post_svpwm - f->t_post_obs);
}
