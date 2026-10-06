/**
 * @file ident_flow.c
 * @date 2026-10-06
 * @brief 堵转辨识编排实现。

 *
 * 节拍限制见 ident_flow.h 文件头。
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
 */

#include "ident_flow.h"

#include <stddef.h>

#include "deadband_service.h"
#include "ident_module.h"
#include "dbg_monitor.h"
#include "motor_params_m1.h"

#if M1_IDENT_ENABLE

typedef enum {
    IDENT_FLOW_DB_OFF = 0,
    IDENT_FLOW_DB_FIXED,
    IDENT_FLOW_DB_LUT,
} ident_flow_db_seg_t;

static ident_module_state_t s_prev_state;
static uint8_t s_prev_round;

static ident_flow_db_seg_t ident_flow_db_seg_from_layout(uint8_t round,
                                                           uint8_t off_rounds,
                                                           uint8_t fixed_rounds)
{
    if (round < off_rounds) {
        return IDENT_FLOW_DB_OFF;
    }
    if (round < (uint8_t)(off_rounds + fixed_rounds)) {
        return IDENT_FLOW_DB_FIXED;
    }
#if M1_DEADBAND_LUT_BAKED_ENABLE || M1_IDENT_ID_CAL_BEFORE_STEP
    return IDENT_FLOW_DB_LUT;
#elif M1_IDENT_STEP_LUT_AFTER_OFF
    return IDENT_FLOW_DB_LUT;
#else
    return IDENT_FLOW_DB_FIXED;
#endif
}

#if M1_IDENT_STEP_BANDS > 1u
static ident_flow_db_seg_t ident_flow_step_db_seg_multi_band(uint8_t round)
{
    const uint8_t pair = round / (uint8_t)M1_IDENT_STEP_ROUNDS_PER_PROFILE;

    return (pair % 2u == 0u) ? IDENT_FLOW_DB_OFF : IDENT_FLOW_DB_LUT;
}
#endif

#if M1_IDENT_BODE_BANDS > 1u
static ident_flow_db_seg_t ident_flow_bode_db_seg_multi_band(uint8_t round)
{
#if !M1_IDENT_BODE_LUT_ENABLE
    (void)round;
    return IDENT_FLOW_DB_OFF;
#else
    return (round % 2u == 0u) ? IDENT_FLOW_DB_OFF : IDENT_FLOW_DB_LUT;
#endif
}
#endif

#if M1_IDENT_IQ_STEP_ENABLE
static ident_flow_db_seg_t ident_flow_step_db_seg(uint8_t round)
{
#if M1_IDENT_STEP_BANDS > 1u
    return ident_flow_step_db_seg_multi_band(round);
#else
    return ident_flow_db_seg_from_layout(round,
                                         (uint8_t)M1_IDENT_STEP_OFF_ROUNDS,
                                         (uint8_t)M1_IDENT_STEP_FIXED_ROUNDS);
#endif
}

static void ident_flow_sync_dbg_step(uint8_t round)
{
#if M1_IDENT_STEP_BANDS > 1u
    /* 双频段 8 轮：round 0..ROUNDS-1 → open_seq 61..(60+ROUNDS)，
     * 与 VOFA ch3/ch4=Iq/Iq_ref 窗口对齐（勿复用 80/81 开环验收相） */
    dbg.open_seq_phase = (uint8_t)(61u + round);
#else
    if (round < M1_IDENT_STEP_OFF_ROUNDS) {
        dbg.open_seq_phase = (uint8_t)(61u + round);
    } else if (round < (M1_IDENT_STEP_OFF_ROUNDS + M1_IDENT_STEP_FIXED_ROUNDS)) {
        dbg.open_seq_phase =
            (uint8_t)(67u + (round - M1_IDENT_STEP_OFF_ROUNDS));
    } else {
        dbg.open_seq_phase = (uint8_t)(74u + (round - M1_IDENT_STEP_OFF_ROUNDS -
                                              M1_IDENT_STEP_FIXED_ROUNDS));
    }
#endif
}
#endif

#if M1_IDENT_IQ_BODE_ENABLE
static ident_flow_db_seg_t ident_flow_bode_db_seg(uint8_t round)
{
#if M1_IDENT_BODE_BANDS > 1u
    return ident_flow_bode_db_seg_multi_band(round);
#else
    return ident_flow_db_seg_from_layout(round,
                                         (uint8_t)M1_IDENT_BODE_OFF_ROUNDS,
                                         (uint8_t)M1_IDENT_BODE_FIXED_ROUNDS);
#endif
}

static void ident_flow_sync_dbg_bode(uint8_t round)
{
    dbg.open_seq_phase = (uint8_t)(62u + round);
}
#endif

static deadband_profile_t ident_flow_db_seg_to_profile(ident_flow_db_seg_t seg)
{
    switch (seg) {
    case IDENT_FLOW_DB_OFF:
        return DEADBAND_PROFILE_OFF;
    case IDENT_FLOW_DB_FIXED:
        return DEADBAND_PROFILE_FIXED;
    case IDENT_FLOW_DB_LUT:
#if M1_IDENT_ID_CAL_BEFORE_STEP
        return DEADBAND_PROFILE_LUT_RUNTIME;
#elif M1_DEADBAND_LUT_BAKED_ENABLE
        return DEADBAND_PROFILE_LUT_BAKED;
#else
        return DEADBAND_PROFILE_OFF;
#endif
    default:
        return DEADBAND_PROFILE_OFF;
    }
}

static void ident_flow_apply_db_seg(ident_flow_db_seg_t seg)
{
    deadband_service_apply_profile(ident_flow_db_seg_to_profile(seg));
}

static void ident_flow_sync_profile(uint8_t force)
{
    const ident_module_state_t state = ident_module_get_state();
    const uint8_t round = ident_module_get_round();

    if (!force &&
        state == s_prev_state &&
        round == s_prev_round) {
        return;
    }

    switch (state) {
#if M1_IDENT_IQ_STEP_ENABLE
    case IDENT_MOD_STEP:
        ident_flow_apply_db_seg(ident_flow_step_db_seg(round));
        break;
#endif
#if M1_IDENT_IQ_BODE_ENABLE
    case IDENT_MOD_BODE:
        ident_flow_apply_db_seg(ident_flow_bode_db_seg(round));
        break;
#endif
    default:
        break;
    }

    s_prev_state = state;
    s_prev_round = round;
}

static void ident_flow_sync_dbg(void)
{
    const ident_module_state_t state = ident_module_get_state();
    const uint8_t round = ident_module_get_round();

    switch (state) {
    case IDENT_MOD_HOLD:
        dbg.open_seq_phase = 60u;
        break;
#if M1_IDENT_IQ_STEP_ENABLE
    case IDENT_MOD_STEP:
        ident_flow_sync_dbg_step(round);
        break;
#endif
#if M1_IDENT_IQ_BODE_ENABLE
    case IDENT_MOD_BODE:
        ident_flow_sync_dbg_bode(round);
        break;
#endif
    case IDENT_MOD_DONE:
    default:
        dbg.open_seq_phase = 73u;
        break;
    }
}

static void ident_flow_sync(motor_context_t *ctx, uint8_t force)
{
    (void)ctx;
    ident_flow_sync_profile(force);
    ident_flow_sync_dbg();
}

/**
 * @brief 初始化堵转辨识编排。
 */
void ident_flow_init(motor_context_t *ctx)
{
    if (ctx == NULL) {
        return;
    }

    deadband_service_reset_runtime();
    s_prev_state = (ident_module_state_t)0xFFu;
    s_prev_round = 0xFFu;
    ident_module_init(ctx);
    ident_flow_sync(ctx, 1u);
}

/**
 * @brief 推进辨识并在换段时切死区。
 */
void ident_flow_tick(motor_context_t *ctx)
{
    if (ctx == NULL) {
        return;
    }

    ident_module_tick(ctx);
    ident_flow_sync(ctx, 0u);
}

static uint8_t ident_flow_db_seg_is_fixed(void)
{
#if M1_IDENT_IQ_STEP_ENABLE
    if (ident_module_get_state() == IDENT_MOD_STEP) {
        return (ident_flow_step_db_seg(ident_module_get_round()) == IDENT_FLOW_DB_FIXED) ?
               1u :
               0u;
    }
#endif
#if M1_IDENT_IQ_BODE_ENABLE
    if (ident_module_get_state() == IDENT_MOD_BODE) {
        return (ident_flow_bode_db_seg(ident_module_get_round()) == IDENT_FLOW_DB_FIXED) ?
               1u :
               0u;
    }
#endif
    return 0u;
}

uint8_t ident_flow_deadband_fixed(void)
{
    return ident_flow_db_seg_is_fixed();
}

#else /* !M1_IDENT_ENABLE */

void ident_flow_init(motor_context_t *ctx)
{
    (void)ctx;
}

void ident_flow_tick(motor_context_t *ctx)
{
    (void)ctx;
}

uint8_t ident_flow_deadband_fixed(void)
{
    return 0u;
}

#endif /* M1_IDENT_ENABLE */
