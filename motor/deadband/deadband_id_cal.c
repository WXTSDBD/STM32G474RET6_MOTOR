/**
 * @file deadband_id_cal.c
 * @date 2026-10-06
 * @brief ALIGN → HOLD → Rs → l_hfi → DONE。建表 Pass0 已删除。

 *
 * 节拍限制见 deadband_id_cal.h 文件头。
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。原 Pass0 见
 *       `_scratch/archive_deadband_lut/deadband_id_cal.c.before_k1`。
 */

#include "deadband_id_cal.h"

#include <stddef.h>

#include "deadband_service.h"
#include "dbg_monitor.h"
#include "foc_pi.h"
#include "motor_cfg.h"
#include "motor_params_m1.h"
#include "rs_ident.h"
#if M1_L_HFI_IDENT_ENABLE
#include "l_hfi_ident.h"
#endif
#if M1_VOFA_IDENT_DUMP_ENABLE
#include "telem_ident_dump.h"
#endif

#if M1_ID_LOCK_CAL_SWEEP

typedef enum {
    M1_ID_CAL_ALIGN = 0,
    M1_ID_CAL_INIT_HOLD,
    M1_ID_CAL_RS_IDENT,
    M1_ID_CAL_L_HFI_PRE_DECAY,
    M1_ID_CAL_L_HFI_IDENT,
    M1_ID_CAL_DONE,
} m1_id_cal_state_t;

static m1_id_cal_state_t s_id_cal_state;
static uint32_t s_id_cal_tick;
static uint8_t s_id_cal_bumpless_arm;
static float s_id_cal_bumpless_id_ref;
static uint8_t s_id_cal_iq_bumpless_arm;
static float s_id_cal_iq_bumpless_ref;
static deadband_id_cal_pass0_cont_fn s_pass0_continue;
#if M1_L_HFI_IDENT_ENABLE
static float s_l_hfi_rs_cached;
#endif

static uint32_t deadband_id_cal_ticks_from_s(float s)
{
    if (s <= 0.0f) {
        return 0u;
    }
    return (uint32_t)(s / M1_CTRL_TS_S + 0.5f);
}

static void deadband_id_cal_arm_bumpless(float id_ref)
{
    s_id_cal_bumpless_arm = 1u;
    s_id_cal_bumpless_id_ref = id_ref;
}

static void deadband_id_cal_arm_bumpless_iq(float iq_ref)
{
    s_id_cal_iq_bumpless_arm = 1u;
    s_id_cal_iq_bumpless_ref = iq_ref;
}

static void deadband_id_cal_sync_open_seq_phase(void)
{
    if (s_id_cal_state == M1_ID_CAL_DONE) {
#if M1_L_HFI_IDENT_ENABLE
        dbg.open_seq_phase = l_hfi_ident_open_seq_phase();
#elif M1_RS_IDENT_ENABLE
        dbg.open_seq_phase = rs_ident_open_seq_phase();
#else
        dbg.open_seq_phase = 9u;
#endif
    } else if (s_id_cal_state == M1_ID_CAL_ALIGN) {
        dbg.open_seq_phase = 38u;
    } else if (s_id_cal_state == M1_ID_CAL_INIT_HOLD) {
        dbg.open_seq_phase = 0u;
#if M1_RS_IDENT_ENABLE
    } else if (s_id_cal_state == M1_ID_CAL_RS_IDENT) {
        dbg.open_seq_phase = rs_ident_open_seq_phase();
#endif
#if M1_L_HFI_IDENT_ENABLE
    } else if (s_id_cal_state == M1_ID_CAL_L_HFI_PRE_DECAY) {
        dbg.open_seq_phase = M1_L_HFI_OPEN_SEQ_PRE_DECAY;
    } else if (s_id_cal_state == M1_ID_CAL_L_HFI_IDENT) {
        dbg.open_seq_phase = l_hfi_ident_open_seq_phase();
#endif
    } else {
        dbg.open_seq_phase = 0u;
    }
}

void deadband_id_cal_sync_dbg(void)
{
    dbg.deadband_cal_len = 0u;
    dbg.deadband_cal_outlier = 0u;
    dbg.deadband_cal_state = 0u;
}

#if M1_ID_CAL_ALIGN_ENABLE && M1_ID_CAL_FIX_THETA_ENABLE
uint8_t deadband_id_cal_use_align_ud(void)
{
    return (s_id_cal_state == M1_ID_CAL_ALIGN) ? 1u : 0u;
}

float deadband_id_cal_align_ud_v(void)
{
    return M1_ID_CAL_ALIGN_UD_V;
}
#endif

uint8_t deadband_id_cal_in_iq_probe(void)
{
    return 0u;
}

#if M1_RS_IDENT_ENABLE
uint8_t deadband_id_cal_in_rs_ident(void)
{
    return (s_id_cal_state == M1_ID_CAL_RS_IDENT) ? 1u : 0u;
}
#endif

#if M1_L_HFI_IDENT_ENABLE
uint8_t deadband_id_cal_in_l_hfi_ident(void)
{
    return (s_id_cal_state == M1_ID_CAL_L_HFI_IDENT) ? 1u : 0u;
}

uint8_t deadband_id_cal_in_l_hfi_pre_decay(void)
{
    return (s_id_cal_state == M1_ID_CAL_L_HFI_PRE_DECAY) ? 1u : 0u;
}

uint8_t deadband_id_cal_use_l_hfi_align_ud(void)
{
    return 0u;
}
#endif

#if M1_ID_CAL_FIX_THETA_ENABLE
uint8_t deadband_id_cal_use_fix_theta(void)
{
#if M1_VOFA_IDENT_DUMP_ENABLE
    if (s_id_cal_state == M1_ID_CAL_DONE && telem_ident_dump_is_busy()) {
        return 1u;
    }
#endif
    return deadband_id_cal_is_running();
}

float deadband_id_cal_target_theta(void)
{
    return M1_ID_CAL_THETA_EL_RAD;
}
#endif

#if M1_ID_CAL_ALIGN_ENABLE && M1_ID_CAL_FIX_THETA_ENABLE && M1_VOFA_IDENT_DUMP_ENABLE
uint8_t deadband_id_cal_use_post_ident_hold_ud(void)
{
    return (s_id_cal_state == M1_ID_CAL_DONE && telem_ident_dump_is_busy()) ? 1u : 0u;
}
#endif

static void deadband_id_cal_enter_done(motor_context_t *ctx)
{
    s_id_cal_state = M1_ID_CAL_DONE;
    deadband_service_apply_profile(DEADBAND_PROFILE_OFF);
#if M1_VOFA_IDENT_DUMP_ENABLE
    telem_ident_dump_arm();
#endif
    ctx->id_ref = 0.0f;
    ctx->iq_ref = M1_ID_CAL_IQ_REF_A;
    foc_pi_reset(&ctx->pi_id);
    foc_pi_reset(&ctx->pi_iq);
    ctx->ud_pi = 0.0f;
    ctx->uq_pi = 0.0f;
    deadband_id_cal_sync_dbg();
}

#if M1_RS_IDENT_ENABLE
static void deadband_id_cal_begin_rs_ident_off(motor_context_t *ctx)
{
    deadband_service_apply_profile(DEADBAND_PROFILE_OFF);
    rs_ident_arm();
    s_id_cal_state = M1_ID_CAL_RS_IDENT;
    s_id_cal_tick = 0u;
    ctx->id_ref = 0.0f;
    ctx->iq_ref = 0.0f;
    foc_pi_reset(&ctx->pi_id);
    foc_pi_reset(&ctx->pi_iq);
    ctx->ud_pi = 0.0f;
    ctx->uq_pi = 0.0f;
    deadband_id_cal_arm_bumpless(0.0f);
    deadband_id_cal_sync_dbg();
    deadband_id_cal_sync_open_seq_phase();
}
#endif

#if M1_L_HFI_IDENT_ENABLE
static void deadband_id_cal_start_l_hfi_ident(motor_context_t *ctx, float rs_ohm)
{
    l_hfi_ident_arm(rs_ohm);
    s_id_cal_state = M1_ID_CAL_L_HFI_IDENT;
    s_id_cal_tick = 0u;
    ctx->id_ref = 0.0f;
    ctx->iq_ref = 0.0f;
    foc_pi_reset(&ctx->pi_id);
    foc_pi_reset(&ctx->pi_iq);
    ctx->ud_pi = 0.0f;
    ctx->uq_pi = 0.0f;
    deadband_id_cal_arm_bumpless(0.0f);
    deadband_id_cal_arm_bumpless_iq(0.0f);
    deadband_id_cal_sync_dbg();
}

static void deadband_id_cal_enter_l_hfi_pre_decay(motor_context_t *ctx)
{
    s_id_cal_state = M1_ID_CAL_L_HFI_PRE_DECAY;
    s_id_cal_tick = 0u;
    ctx->id_ref = 0.5f;
    ctx->iq_ref = 0.0f;
    foc_pi_reset(&ctx->pi_id);
    foc_pi_reset(&ctx->pi_iq);
    ctx->ud_pi = 0.0f;
    ctx->uq_pi = 0.0f;
    deadband_id_cal_arm_bumpless(0.5f);
    deadband_id_cal_arm_bumpless_iq(0.0f);
    deadband_id_cal_sync_dbg();
}

static void deadband_id_cal_cache_rs_for_l_hfi(void)
{
    rs_ident_result_t rs;

    s_l_hfi_rs_cached = g_m1_motor_cfg.rs_ohm;
    rs_ident_get_result(&rs);
    if (rs.ok != 0u && rs.rs_ohm > 0.0f) {
        s_l_hfi_rs_cached = rs.rs_ohm;
    }
}

static void deadband_id_cal_begin_l_hfi_after_rs(motor_context_t *ctx)
{
    deadband_id_cal_cache_rs_for_l_hfi();
    deadband_id_cal_enter_l_hfi_pre_decay(ctx);
}

#if M1_RS_IDENT_USE_FIXED_NOMINAL
static void deadband_id_cal_begin_l_hfi_with_fixed_rs(motor_context_t *ctx)
{
    deadband_service_apply_profile(DEADBAND_PROFILE_OFF);
    s_l_hfi_rs_cached = g_m1_motor_cfg.rs_ohm;
    ctx->id_ref = 0.0f;
    ctx->iq_ref = 0.0f;
    foc_pi_reset(&ctx->pi_id);
    foc_pi_reset(&ctx->pi_iq);
    ctx->ud_pi = 0.0f;
    ctx->uq_pi = 0.0f;
    deadband_id_cal_arm_bumpless(0.0f);
    deadband_id_cal_arm_bumpless_iq(0.0f);
    deadband_id_cal_enter_l_hfi_pre_decay(ctx);
    deadband_id_cal_sync_dbg();
    deadband_id_cal_sync_open_seq_phase();
}
#endif
#endif /* M1_L_HFI_IDENT_ENABLE */

void deadband_id_cal_set_pass0_continue(deadband_id_cal_pass0_cont_fn fn)
{
    s_pass0_continue = fn;
}

void deadband_id_cal_continue_rs_lhfi(motor_context_t *ctx)
{
    if (ctx == NULL) {
        return;
    }
#if M1_RS_IDENT_ENABLE
    deadband_id_cal_begin_rs_ident_off(ctx);
#elif M1_L_HFI_IDENT_ENABLE && M1_RS_IDENT_USE_FIXED_NOMINAL
    deadband_id_cal_begin_l_hfi_with_fixed_rs(ctx);
#else
    deadband_id_cal_enter_done(ctx);
#endif
}

void deadband_id_cal_init(void)
{
#if M1_ID_CAL_ALIGN_ENABLE && M1_ID_CAL_FIX_THETA_ENABLE
    s_id_cal_state = M1_ID_CAL_ALIGN;
#else
    s_id_cal_state = M1_ID_CAL_INIT_HOLD;
#endif
    s_id_cal_tick = 0u;
    s_id_cal_bumpless_arm = 0u;
    s_id_cal_bumpless_id_ref = 0.0f;
    s_id_cal_iq_bumpless_arm = 0u;
    s_id_cal_iq_bumpless_ref = 0.0f;
    s_pass0_continue = NULL;
#if M1_RS_IDENT_ENABLE
    rs_ident_init();
#endif
#if M1_L_HFI_IDENT_ENABLE
    l_hfi_ident_init();
#endif
    deadband_id_cal_sync_dbg();
    deadband_id_cal_sync_open_seq_phase();
}

#if M1_ID_CAL_RS_LD_LQ_ONLY_ENABLE
/**
 * @brief 跳过建表：死区 OFF；随后 INIT_HOLD → Rs → l_hfi。
 */
void deadband_id_cal_boot_rs_l_hfi_only(motor_context_t *ctx)
{
    if (ctx == NULL) {
        return;
    }
    deadband_service_apply_profile(DEADBAND_PROFILE_OFF);
#if M1_L_HFI_IDENT_ENABLE
    l_hfi_ident_init();
#endif
}
#endif

void deadband_id_cal_tick(motor_context_t *ctx)
{
    if (ctx == NULL) {
        return;
    }

    if (!deadband_id_cal_in_l_hfi_ident() && !deadband_id_cal_in_rs_ident() &&
        !deadband_id_cal_in_l_hfi_pre_decay()) {
        ctx->iq_ref = M1_ID_CAL_IQ_REF_A;
    }

    switch (s_id_cal_state) {
#if M1_ID_CAL_ALIGN_ENABLE && M1_ID_CAL_FIX_THETA_ENABLE
    case M1_ID_CAL_ALIGN: {
        const uint32_t align_ticks = deadband_id_cal_ticks_from_s(M1_ID_CAL_ALIGN_S);

        ctx->id_ref = 0.0f;
        s_id_cal_tick++;
        if (align_ticks == 0u || s_id_cal_tick >= align_ticks) {
            s_id_cal_state = M1_ID_CAL_INIT_HOLD;
            s_id_cal_tick = 0u;
            foc_pi_reset(&ctx->pi_id);
            foc_pi_reset(&ctx->pi_iq);
            ctx->ud_pi = 0.0f;
            ctx->uq_pi = 0.0f;
        }
        break;
    }
#endif

    case M1_ID_CAL_INIT_HOLD: {
        const uint32_t hold_ticks =
            deadband_id_cal_ticks_from_s(M1_ID_CAL_INIT_HOLD_S);

        ctx->id_ref = 0.0f;
        s_id_cal_tick++;
        if (s_id_cal_tick >= hold_ticks) {
#if M1_RS_IDENT_USE_FIXED_NOMINAL && M1_L_HFI_IDENT_ENABLE
            deadband_id_cal_begin_l_hfi_with_fixed_rs(ctx);
#elif M1_RS_IDENT_ENABLE
            deadband_id_cal_begin_rs_ident_off(ctx);
#else
#error "id_cal slim path needs RS_IDENT or FIXED_NOMINAL+L_HFI"
#endif
        }
        break;
    }

#if M1_RS_IDENT_ENABLE
    case M1_ID_CAL_RS_IDENT:
        ctx->iq_ref = 0.0f;
        ctx->id_ref = deadband_id_cal_clamp_id_ref(
            rs_ident_tick(ctx->id, ctx->ud_pi));
        if (rs_ident_is_done()) {
#if M1_L_HFI_IDENT_ENABLE
            deadband_id_cal_begin_l_hfi_after_rs(ctx);
#else
            deadband_id_cal_enter_done(ctx);
#endif
        }
        break;
#endif

#if M1_L_HFI_IDENT_ENABLE
    case M1_ID_CAL_L_HFI_PRE_DECAY: {
        const uint32_t decay_ticks =
            deadband_id_cal_ticks_from_s(M1_L_HFI_PRE_DECAY_S);

        /* 保一点 Id，避免 0.8s 零电流后转子滑离 FIX_THETA 贴角 */
        ctx->id_ref = 0.5f;
        ctx->iq_ref = 0.0f;
        s_id_cal_tick++;
        if (decay_ticks == 0u || s_id_cal_tick >= decay_ticks) {
            deadband_id_cal_start_l_hfi_ident(ctx, s_l_hfi_rs_cached);
        }
        break;
    }

    case M1_ID_CAL_L_HFI_IDENT:
        l_hfi_ident_tick(ctx->id, ctx->iq);
        ctx->id_ref = 0.0f;
        ctx->iq_ref = 0.0f;
        if (l_hfi_ident_is_done() != 0u) {
            l_hfi_ident_sync_dbg();
            foc_pi_reset(&ctx->pi_id);
            foc_pi_reset(&ctx->pi_iq);
            ctx->ud_pi = 0.0f;
            ctx->uq_pi = 0.0f;
            deadband_id_cal_enter_done(ctx);
        }
        break;
#endif

    case M1_ID_CAL_DONE:
    default:
        ctx->id_ref = 0.0f;
        ctx->iq_ref = M1_ID_CAL_IQ_REF_A;
        break;
    }

    deadband_id_cal_sync_open_seq_phase();
}

uint8_t deadband_id_cal_is_running(void)
{
    return (s_id_cal_state != M1_ID_CAL_DONE) ? 1u : 0u;
}

uint8_t deadband_id_cal_is_done(void)
{
    return (s_id_cal_state == M1_ID_CAL_DONE) ? 1u : 0u;
}

uint8_t deadband_id_cal_use_cal_pi_limits(void)
{
    return deadband_id_cal_is_running();
}

float deadband_id_cal_clamp_id_ref(float ref)
{
    if (ref > M1_ID_CAL_I_REF_ABS_MAX) {
        return M1_ID_CAL_I_REF_ABS_MAX;
    }
    if (ref < -M1_ID_CAL_I_REF_ABS_MAX) {
        return -M1_ID_CAL_I_REF_ABS_MAX;
    }
    return ref;
}

float deadband_id_cal_clamp_iq_ref(float ref)
{
    return deadband_id_cal_clamp_id_ref(ref);
}

uint8_t deadband_id_cal_bumpless_arm(void)
{
    return s_id_cal_bumpless_arm;
}

float deadband_id_cal_bumpless_id_ref(void)
{
    return s_id_cal_bumpless_id_ref;
}

void deadband_id_cal_consume_bumpless_arm(void)
{
    s_id_cal_bumpless_arm = 0u;
}

uint8_t deadband_id_cal_iq_bumpless_arm(void)
{
    return s_id_cal_iq_bumpless_arm;
}

float deadband_id_cal_iq_bumpless_ref(void)
{
    return s_id_cal_iq_bumpless_ref;
}

void deadband_id_cal_consume_iq_bumpless_arm(void)
{
    s_id_cal_iq_bumpless_arm = 0u;
}

uint8_t deadband_id_cal_should_capture(void)
{
    return 0u;
}

float deadband_id_cal_capture_id_ref(void)
{
    return 0.0f;
}

void deadband_id_cal_clear_capture_pending(void)
{
}

#else /* !M1_ID_LOCK_CAL_SWEEP */

void deadband_id_cal_init(void) {}
void deadband_id_cal_tick(motor_context_t *ctx) { (void)ctx; }
uint8_t deadband_id_cal_is_running(void) { return 0u; }
uint8_t deadband_id_cal_is_done(void) { return 0u; }
uint8_t deadband_id_cal_in_iq_probe(void) { return 0u; }
uint8_t deadband_id_cal_use_cal_pi_limits(void) { return 0u; }
float deadband_id_cal_clamp_id_ref(float ref) { return ref; }
float deadband_id_cal_clamp_iq_ref(float ref) { return ref; }
uint8_t deadband_id_cal_bumpless_arm(void) { return 0u; }
float deadband_id_cal_bumpless_id_ref(void) { return 0.0f; }
void deadband_id_cal_consume_bumpless_arm(void) {}
uint8_t deadband_id_cal_iq_bumpless_arm(void) { return 0u; }
float deadband_id_cal_iq_bumpless_ref(void) { return 0.0f; }
void deadband_id_cal_consume_iq_bumpless_arm(void) {}
uint8_t deadband_id_cal_should_capture(void) { return 0u; }
float deadband_id_cal_capture_id_ref(void) { return 0.0f; }
void deadband_id_cal_clear_capture_pending(void) {}
void deadband_id_cal_sync_dbg(void) {}
void deadband_id_cal_set_pass0_continue(deadband_id_cal_pass0_cont_fn fn)
{
    (void)fn;
}
void deadband_id_cal_continue_rs_lhfi(motor_context_t *ctx) { (void)ctx; }

#endif /* M1_ID_LOCK_CAL_SWEEP */
