/**
 * @file isr_entry.c
 * @date 2026-10-29
 * @brief 20 kHz FOC 入口编排：固定顺序 sample→startup→current→modulate→telem。
 *
 * 本文件先承接原 motor_current_tick 整段，再逐步把块下沉到同目录其它 isr_*.c。
 * 不新增选路宏。
 */
#include "isr/isr_frame.h"
#include "isr/isr_state.h"

#include <stddef.h>

#include "stm32g474xx.h"

#include "encoder.h"
#include "dbg_monitor.h"
#include "foc_svpwm.h"
#include "motor_current.h"
#include "motor_current_exp_hooks.h"
#include "motor_outer_loop.h"
#include "motor_phase_binding.h"
#include "motor_params_m1.h"
#include "core/motor_route.h"
#include "adc_sample.h"

#include "motor_pll.h"
#include "observer/observer_composite.h"

#define M1_PLL_PRIME_TICKS   200u

static float motor_current_uq_for_sector(const motor_context_t *ctx)
{
    if (ctx->mode == M1_CTRL_CURRENT_LOOP) {
        return ctx->uq_pi;
    }
    return ctx->uq_open;
}

static float motor_current_ud_for_sector(const motor_context_t *ctx)
{
    if (ctx->mode == M1_CTRL_CURRENT_LOOP) {
        return ctx->ud_pi;
    }
    return 0.0f;
}

static void motor_current_reconstruct_abc(const motor_context_t *ctx,
                                           float theta_svpwm,
                                           float *ia, float *ib, float *ic)
{
    float Uq;
    float Ud;
    int sec;
    float a;
    float b;
    float c;

    if (ctx->mode == M1_CTRL_CURRENT_LOOP) {
        return;
    }

    Uq = motor_current_uq_for_sector(ctx);
    Ud = motor_current_ud_for_sector(ctx);
    sec = svpwm_sector_from_uq_ud(Uq, Ud, theta_svpwm);
    if (sec != 1 && sec != 2) {
        return;
    }

    a = (*ia >= 0.0f) ? *ia : -*ia;
    b = (*ib >= 0.0f) ? *ib : -*ib;
    c = (*ic >= 0.0f) ? *ic : -*ic;
    if (a < M1_CURRENT_RECON_MIN_A && b < M1_CURRENT_RECON_MIN_A &&
        c < M1_CURRENT_RECON_MIN_A) {
        return;
    }

    *ic = -(*ia + *ib);
}


void isr_entry_tick(bsp_axis_t *axis)
{
    motor_context_t *ctx;
    volatile uint32_t isr_t0 = *(volatile uint32_t *)&DWT->CYCCNT;
    uint16_t enc_raw;
    float i_alpha;
    float i_beta;
    float ia;
    float ib;
    float ic;
    float theta;
    float theta_enc_park;
    volatile uint32_t foc_t0;

    if (axis == NULL || axis->enc == NULL || axis->pwm == NULL) {
        return;
    }

    ctx = motor_current_ctx(axis);
    if (ctx == NULL) {
        return;
    }

    if (M1_SPEED_LOOP_ENABLE) {
        g_isr_diag_foc_cnt++;
    }

    if (axis->adc.cal_active) {
        return;
    }

    enc_raw = encoder_get_raw(axis->enc);
    dbg.enc_raw = (float)enc_raw;
    g_isr_theta_mech_rad =
        M1_ENCODER_MECH_SIGN * encoder_get_angle(axis->enc, enc_raw);
    theta = encoder_get_theta_el(axis->enc, enc_raw, ctx->pole_pairs,
                                 encoder_get_theta_el_offset(axis->enc));
    if (M1_PLL_ENABLE) {
        const float theta_mech = g_isr_theta_mech_rad;

        if (g_isr_pll_prime_ticks < (uint16_t)M1_PLL_PRIME_TICKS) {
            g_isr_pll_prime_ticks++;
            motor_pll_reset(&g_isr_m1_pll, theta_mech);
        } else {
            motor_pll_update(&g_isr_m1_pll, theta_mech, M1_CTRL_TS_S);
        }
        g_isr_pll_omega_mech_rpm = motor_pll_get_omega_mech_rpm(&g_isr_m1_pll);
        g_isr_theta_mech_pll_rad = motor_pll_get_theta(&g_isr_m1_pll);
        g_isr_speed_fb_rpm = g_isr_pll_omega_mech_rpm;
        if (M1_HFI_ENABLE && M1_HFI_SPEED_FB_ENABLE &&
            M1_HFI_MOTION_BYPASS_ENABLE) {
            if (motor_angle_active() != NULL) {
                g_isr_speed_fb_rpm =
                    motor_angle_active()->get_omega_rpm(&g_isr_m1_ctx);
            }
        }
    }
    theta_enc_park = theta;
    dbg.enc_theta_el = theta_enc_park;

    {
        isr_frame_t sample;
        float i_phys[3];

        sample.axis = axis;
        sample.ctx = ctx;
        adc_sample_get_abc(&axis->adc, &i_phys[0], &i_phys[1], &i_phys[2]);
        motor_phase_binding_map_abc(i_phys, &sample.ia, &sample.ib, &sample.ic);
        if (M1_CURRENT_RECON_ENABLE) {
            if (M1_HFI_ENABLE && M1_HFI_MOTION_BYPASS_ENABLE) {
                if (g_isr_hfi_recon_theta_ok != 0u) {
                    motor_current_reconstruct_abc(ctx, g_isr_hfi_recon_theta,
                                                  &sample.ia, &sample.ib,
                                                  &sample.ic);
                }
            } else {
                motor_current_reconstruct_abc(ctx, theta_enc_park,
                                              &sample.ia, &sample.ib,
                                              &sample.ic);
            }
        }
        isr_sample_run(&sample);
        ia = sample.ia;
        ib = sample.ib;
        ic = sample.ic;
        i_alpha = sample.i_alpha;
        i_beta = sample.i_beta;
    }
    foc_t0 = *(volatile uint32_t *)&DWT->CYCCNT;

    motor_current_exp_flow_tick(ctx);

    if (M1_SPEED_LOOP_ENABLE) {
        /* 外环在 TIM6 2 kHz（motor_current_outer_tick_2k）；此处只置使能。 */
        g_isr_outer_run_enable = 1u;
    }

    {
        isr_frame_t frame;

        frame.axis = axis;
        frame.ctx = ctx;
        frame.ia = ia;
        frame.ib = ib;
        frame.ic = ic;
        frame.i_alpha = i_alpha;
        frame.i_beta = i_beta;
        frame.theta_enc_park = theta_enc_park;
        frame.theta_park = 0.0f;
        frame.ua_hfi = 0.0f;
        frame.ub_hfi = 0.0f;
        frame.isr_t0 = isr_t0;
        frame.foc_t0 = foc_t0;
        isr_startup_run(&frame);
        isr_current_run(&frame);
        isr_obs_run(&frame);
        isr_modulate_run(&frame);
        frame.t_post_svpwm = *(volatile uint32_t *)&DWT->CYCCNT;
        isr_telem_run(&frame);
    }
}
