/**
 * @file motor_current_exp_hooks.c
 * @date 2026-10-19
 * @brief 电流环实验旁路：辨识/开环阶梯；宏关时走空路径。
 *
 * 节拍限制见 motor_current_exp_hooks.h 文件头。
 * 能力闸门用 if (M1_*)，不再用 #if。
 */

#include "motor_current_exp_hooks.h"

#include "deadband_flow.h"
#include "deadband_id_cal.h"
#include "ident_flow.h"
#include "l_hfi_ident.h"
#include "motor_params_m1.h"

static uint8_t s_ident_booted;
static uint8_t s_db_flow_booted;

void motor_current_exp_flow_tick(motor_context_t *ctx)
{
    if (M1_IDENT_ENABLE) {
        if (ctx == 0) {
            return;
        }
        if (s_ident_booted == 0u) {
            ident_flow_init(ctx);
            s_ident_booted = 1u;
        }
        ident_flow_tick(ctx);
        return;
    }
    if (M1_DEADBAND_FLOW_ENABLE) {
        if (ctx == 0) {
            return;
        }
        if (s_db_flow_booted == 0u) {
            deadband_flow_boot(ctx);
            s_db_flow_booted = 1u;
        }
        deadband_flow_tick(ctx);
        return;
    }
    (void)ctx;
}

uint8_t motor_current_exp_try_ld_lq_udq(motor_context_t *ctx, float id,
                                       float iq, float *ud, float *uq)
{
    if (!M1_L_HFI_IDENT_ENABLE) {
        (void)ctx;
        (void)id;
        (void)iq;
        (void)ud;
        (void)uq;
        return 0u;
    }
    if ((ctx == 0) || (ud == 0) || (uq == 0)) {
        return 0u;
    }
    if (deadband_id_cal_in_l_hfi_ident() == 0u) {
        return 0u;
    }
    if (l_hfi_ident_is_done() != 0u) {
        return 0u;
    }
    if (l_hfi_ident_inject_active() != 0u) {
        l_hfi_ident_on_sample(id, iq);
    }
    *ud = l_hfi_ident_u_d();
    *uq = l_hfi_ident_u_q();
    return 1u;
}

uint8_t motor_current_exp_try_open_udq(motor_context_t *ctx, float *ud,
                                      float *uq)
{
    (void)ctx;
    (void)ud;
    (void)uq;
    return 0u;
}
