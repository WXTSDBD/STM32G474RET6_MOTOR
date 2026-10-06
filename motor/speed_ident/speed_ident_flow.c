/**
 * @file speed_ident_flow.c
 * @date 2026-10-06
 * @brief 速度环辨识编排实现。

 *
 * 节拍限制见 speed_ident_flow.h 文件头。
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
 */

#include "speed_ident_flow.h"

#include <stddef.h>

#include "deadband_service.h"
#include "dbg_monitor.h"
#include "foc_pi.h"
#include "motor_current.h"
#include "motor_outer_loop.h"
#include "motor_params_m1.h"
#include "speed_ident_module.h"

#if M1_SPEED_IDENT_ENABLE

static speed_ident_module_state_t s_prev_state;
static uint8_t s_prev_round;
static uint8_t s_speed_ident_armed;

static void speed_ident_flow_sync_dbg(void)
{
    const speed_ident_module_state_t state = speed_ident_module_get_state();
    const uint8_t round = speed_ident_module_get_round();

    switch (state) {
    case SPEED_IDENT_HOLD:
        dbg.open_seq_phase = 220u;
        break;
#if M1_SPEED_IDENT_STEP_ENABLE
    case SPEED_IDENT_STEP:
        /* 221..226 = STEP 第 0..5 相（与 motor_params 注释一致） */
        dbg.open_seq_phase =
            (uint8_t)(221u + speed_ident_module_get_phase_in_round());
        break;
#endif
#if M1_SPEED_IDENT_BODE_ENABLE
    case SPEED_IDENT_BODE:
        dbg.open_seq_phase = 230u;
        break;
#endif
    case SPEED_IDENT_DONE:
    default:
        dbg.open_seq_phase = 239u;
        break;
    }

    (void)round;
}

static void speed_ident_flow_sync(uint8_t force)
{
    const speed_ident_module_state_t state = speed_ident_module_get_state();
    const uint8_t round = speed_ident_module_get_round();

    if (!force && state == s_prev_state && round == s_prev_round) {
        speed_ident_flow_sync_dbg();
        return;
    }

    deadband_service_apply_profile(DEADBAND_PROFILE_OFF);
    s_prev_state = state;
    s_prev_round = round;
    speed_ident_flow_sync_dbg();
}

/**
 * @brief 初始化速度辨识编排。
 */
void speed_ident_flow_init(motor_context_t *ctx)
{
    if (ctx == NULL) {
        return;
    }

    deadband_service_reset_runtime();
    deadband_service_apply_profile(DEADBAND_PROFILE_OFF);
    s_prev_state = (speed_ident_module_state_t)0xFFu;
    s_prev_round = 0xFFu;
    s_speed_ident_armed = 1u;
    speed_ident_module_init(ctx);
    {
        const float omega_now = motor_current_get_pll_omega_mech_rpm();
        const float iq_meas = 0.0f;

        if (ctx->outer_mode != M1_OUTER_SPEED) {
            motor_outer_set_mode(ctx, M1_OUTER_SPEED, iq_meas, omega_now);
        } else {
            motor_outer_sync_speed_boot(ctx, iq_meas, omega_now);
        }
    }
    ctx->iq_ref = 0.0f;
    speed_ident_flow_sync(1u);
}

/**
 * @brief 推进速度辨识。
 */
void speed_ident_flow_tick(motor_context_t *ctx)
{
    if (ctx == NULL) {
        return;
    }

    speed_ident_module_tick(ctx);
    speed_ident_flow_sync(0u);
}

uint8_t speed_ident_flow_is_armed(void)
{
    return s_speed_ident_armed;
}

#else /* !M1_SPEED_IDENT_ENABLE */

void speed_ident_flow_init(motor_context_t *ctx)
{
    (void)ctx;
}

void speed_ident_flow_tick(motor_context_t *ctx)
{
    (void)ctx;
}

uint8_t speed_ident_flow_is_armed(void)
{
    return 0u;
}

#endif /* M1_SPEED_IDENT_ENABLE */
