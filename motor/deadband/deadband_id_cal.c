/**
 * @file deadband_id_cal.c
 * @brief Id 锁轴扫表状态机（Pass0 capture / commit / Iq 探路）。
 */

#include "deadband_id_cal.h"

#include "deadband_cal.h"
#include "deadband_service.h"
#include "foc_pi.h"
#include "dbg_monitor.h"
#include "motor_params_m1.h"
#include "telem_lut_dump.h"

#if M1_ID_LOCK_CAL_SWEEP

float deadband_id_cal_clamp_id_ref(float ref);

/** Pass0：0.05～0.30 A ×16，0.35～1.00 A ×12，1.10～1.50 A ×4（4:3:1） */
static const float s_id_cal_amp_table[M1_ID_CAL_AMP_TABLE_LEN] = {
    0.05f,  0.0667f, 0.0833f, 0.10f,  0.1167f, 0.1333f, 0.15f,  0.1667f,
    0.1833f, 0.20f,  0.2167f, 0.2333f, 0.25f,  0.2667f, 0.2833f, 0.30f,
    0.35f,  0.4091f, 0.4682f, 0.5273f, 0.5864f, 0.6455f, 0.7045f, 0.7636f,
    0.8227f, 0.8818f, 0.9409f, 1.00f,  1.10f,  1.2333f, 1.3667f, 1.50f,
};

#if M1_ID_CAL_LUT_VERIFY_SWEEP && M1_ID_CAL_COMMIT_LUT
/** Pass1 续扫：1.50 A 之后按 0.1333 A/档至 M1_ID_CAL_VERIFY_I_MAX_A */
static const float s_id_cal_verify_ext_table[M1_ID_CAL_VERIFY_EXT_LEN] = {
    1.6333f, 1.7667f, 1.9000f, 2.0333f, 2.1667f, 2.3000f,
    2.4333f, 2.5667f, 2.7000f, 2.8333f, 3.0000f,
};
#endif

typedef enum {
    M1_ID_CAL_ALIGN,
    M1_ID_CAL_INIT_HOLD,
    M1_ID_CAL_ID_STEP,
    M1_ID_CAL_PASS0A_DECAY,
    M1_ID_CAL_ALIGN_B,
    M1_ID_CAL_PASS0_DECAY,
    M1_ID_CAL_IQ_PROBE_OFF,
    M1_ID_CAL_IQ_PROBE_FIXED,
    M1_ID_CAL_IQ_PROBE_ON,
    M1_ID_CAL_DONE,
} m1_id_cal_state_t;

static m1_id_cal_state_t s_id_cal_state;
static uint32_t s_id_cal_tick;
static uint8_t s_id_cal_id_step;
static uint8_t s_id_cal_bumpless_arm;
static float s_id_cal_bumpless_id_ref;
static uint8_t s_id_cal_iq_bumpless_arm;
static float s_id_cal_iq_bumpless_ref;
static uint8_t s_id_cal_capture_pending;
static float s_id_cal_capture_id_ref;
static uint8_t s_id_cal_capture_armed;
/** 0=标定(OFF+capture)，1=LUT 验收扫表(ON，不 capture) */
static uint8_t s_id_cal_pass;
#if M1_ID_CAL_DUAL_ANGLE_ENABLE
/** Pass0 角序号：双角 0=30° 1=0°；MULTI 时 0..N-1 见 s_id_cal_pass0_theta_rad[] */
static uint8_t s_id_cal_pass0_leg;
#if M1_ID_CAL_MULTI_ANGLE_ENABLE
static const float s_id_cal_pass0_theta_rad[M1_ID_CAL_PASS0_ANGLE_COUNT] = {
    M1_ID_CAL_THETA_PASS0_0_RAD,
    M1_ID_CAL_THETA_PASS0_A_RAD,
    M1_ID_CAL_THETA_PASS0_60_RAD,
    M1_ID_CAL_THETA_PASS0_90_RAD,
    M1_ID_CAL_THETA_PASS0_120_RAD,
};
#endif
#endif

#if M1_ID_CAL_ALIGN_ENABLE && M1_ID_CAL_FIX_THETA_ENABLE
/** 仅 ALIGN 态开环 Ud；DECAY 须 Id=0 闭环泄流，禁止再叠 3V（0° 下过流） */
uint8_t deadband_id_cal_use_align_ud(void)
{
#if M1_ID_CAL_MULTI_ANGLE_ENABLE
    return (s_id_cal_state == M1_ID_CAL_ALIGN);
#else
    return (s_id_cal_state == M1_ID_CAL_ALIGN) ||
           (s_id_cal_state == M1_ID_CAL_ALIGN_B);
#endif
}
#endif

uint8_t deadband_id_cal_in_iq_probe(void)
{
    return (s_id_cal_state == M1_ID_CAL_IQ_PROBE_OFF) ||
           (s_id_cal_state == M1_ID_CAL_IQ_PROBE_FIXED) ||
           (s_id_cal_state == M1_ID_CAL_IQ_PROBE_ON);
}

#if M1_ID_CAL_FIX_THETA_ENABLE
uint8_t deadband_id_cal_use_fix_theta(void)
{
    return !deadband_id_cal_in_iq_probe();
}
#endif

static void deadband_id_cal_sync_open_seq_phase(void)
{
    if (s_id_cal_state == M1_ID_CAL_DONE) {
        dbg.open_seq_phase = 9u;
    } else if (s_id_cal_state == M1_ID_CAL_IQ_PROBE_OFF) {
        dbg.open_seq_phase = 50u;
    } else if (s_id_cal_state == M1_ID_CAL_IQ_PROBE_FIXED) {
        dbg.open_seq_phase = 53u;
    } else if (s_id_cal_state == M1_ID_CAL_IQ_PROBE_ON) {
        dbg.open_seq_phase = 51u;
#if M1_ID_CAL_DUAL_ANGLE_ENABLE
    } else if (s_id_cal_state == M1_ID_CAL_PASS0A_DECAY) {
        dbg.open_seq_phase = 35u;
    } else if (s_id_cal_state == M1_ID_CAL_ALIGN_B) {
        dbg.open_seq_phase = 52u;
#endif
    } else if (s_id_cal_state == M1_ID_CAL_PASS0_DECAY) {
        dbg.open_seq_phase = 39u;
    } else if (s_id_cal_state == M1_ID_CAL_ALIGN) {
#if M1_ID_CAL_MULTI_ANGLE_ENABLE
        dbg.open_seq_phase = (uint8_t)(118u + s_id_cal_pass0_leg);
#else
        dbg.open_seq_phase = 38u;
#endif
    } else if (s_id_cal_state == M1_ID_CAL_INIT_HOLD) {
        dbg.open_seq_phase = s_id_cal_pass ? 40u : 0u;
    } else {
#if M1_ID_CAL_DUAL_ANGLE_ENABLE
#if M1_ID_CAL_MULTI_ANGLE_ENABLE
        if (s_id_cal_pass == 0u) {
            dbg.open_seq_phase =
                (uint8_t)(120u + s_id_cal_pass0_leg * 50u + s_id_cal_id_step);
        } else {
            dbg.open_seq_phase =
                (uint8_t)(41u + s_id_cal_id_step);
        }
#else
        if (s_id_cal_pass == 0u && s_id_cal_pass0_leg != 0u) {
            dbg.open_seq_phase = (uint8_t)(101u + s_id_cal_id_step);
        } else {
            dbg.open_seq_phase = s_id_cal_pass ?
                (uint8_t)(41u + s_id_cal_id_step) :
                (uint8_t)(s_id_cal_id_step + 1u);
        }
#endif
#else
        dbg.open_seq_phase = s_id_cal_pass ?
            (uint8_t)(41u + s_id_cal_id_step) :
            (uint8_t)(s_id_cal_id_step + 1u);
#endif
    }
}

void deadband_id_cal_sync_dbg(void)
{
    dbg.deadband_cal_len = deadband_cal_len();
    dbg.deadband_cal_outlier = deadband_cal_outlier_seen() ? 1u : 0u;
    dbg.deadband_cal_state = (uint8_t)deadband_cal_get_state();
}

static uint32_t deadband_id_cal_ticks_from_s(float s)
{
    if (s <= 0.0f) {
        return 0u;
    }
    return (uint32_t)(s / M1_CTRL_TS_S + 0.5f);
}

#if M1_ID_CAL_DUAL_ANGLE_ENABLE && M1_ID_CAL_FIX_THETA_ENABLE
static uint8_t deadband_id_cal_pass0_angle_count(void)
{
#if M1_ID_CAL_MULTI_ANGLE_ENABLE
    return (uint8_t)M1_ID_CAL_PASS0_ANGLE_COUNT;
#else
    return 2u;
#endif
}

static float deadband_id_cal_pass0_theta_for_leg(uint8_t leg)
{
#if M1_ID_CAL_MULTI_ANGLE_ENABLE
    if (leg >= M1_ID_CAL_PASS0_ANGLE_COUNT) {
        leg = (uint8_t)(M1_ID_CAL_PASS0_ANGLE_COUNT - 1u);
    }
    return s_id_cal_pass0_theta_rad[leg];
#else
    return (leg == 0u) ? M1_ID_CAL_THETA_PASS0_A_RAD : M1_ID_CAL_THETA_PASS0_B_RAD;
#endif
}

#if M1_ID_CAL_DUAL_ANGLE_ENABLE
uint8_t deadband_id_cal_capture_append_dlut(void)
{
#if M1_ID_CAL_MULTI_ANGLE_ENABLE
    {
        const float th = deadband_id_cal_pass0_theta_for_leg(s_id_cal_pass0_leg);
        const float dth = th - M1_ID_CAL_THETA_PASS0_A_RAD;

        return (dth >= -0.02f && dth <= 0.02f) ? 1u : 0u;
    }
#else
    return (s_id_cal_pass0_leg == 0u) ? 1u : 0u;
#endif
}
#endif /* capture append */

float deadband_id_cal_target_theta(void)
{
    if (s_id_cal_pass != 0u) {
        return M1_ID_CAL_THETA_PASS0_A_RAD;
    }
#if M1_ID_CAL_MULTI_ANGLE_ENABLE
    return deadband_id_cal_pass0_theta_for_leg(s_id_cal_pass0_leg);
#else
    if (s_id_cal_state == M1_ID_CAL_ALIGN_B ||
        (s_id_cal_state != M1_ID_CAL_ALIGN && s_id_cal_pass0_leg != 0u)) {
        return M1_ID_CAL_THETA_PASS0_B_RAD;
    }
    return M1_ID_CAL_THETA_PASS0_A_RAD;
#endif
}

#if M1_ID_CAL_ALIGN_ENABLE
float deadband_id_cal_align_ud_v(void)
{
    if (s_id_cal_state == M1_ID_CAL_ALIGN_B) {
        return M1_ID_CAL_ALIGN_B_UD_V;
    }
    return M1_ID_CAL_ALIGN_UD_V;
}

static uint32_t deadband_id_cal_align_ticks(void)
{
    if (s_id_cal_state == M1_ID_CAL_ALIGN_B) {
        return deadband_id_cal_ticks_from_s(M1_ID_CAL_ALIGN_B_S);
    }
    return deadband_id_cal_ticks_from_s(M1_ID_CAL_ALIGN_S);
}
#endif /* M1_ID_CAL_ALIGN_ENABLE */
#endif /* DUAL_ANGLE + FIX_THETA */

static uint8_t deadband_id_cal_step_count(void)
{
#if M1_ID_CAL_LUT_VERIFY_SWEEP && M1_ID_CAL_COMMIT_LUT
    if (s_id_cal_pass != 0u) {
        return (uint8_t)M1_ID_CAL_VERIFY_AMP_TABLE_LEN;
    }
#endif
    return (uint8_t)M1_ID_CAL_AMP_TABLE_LEN;
}

static float deadband_id_cal_ref_from_step(uint8_t step)
{
#if M1_ID_CAL_LUT_VERIFY_SWEEP && M1_ID_CAL_COMMIT_LUT
    if (s_id_cal_pass != 0u) {
        if (step < M1_ID_CAL_AMP_TABLE_LEN) {
            return deadband_id_cal_clamp_id_ref(s_id_cal_amp_table[step]);
        }
        step = (uint8_t)(step - M1_ID_CAL_AMP_TABLE_LEN);
        if (step >= M1_ID_CAL_VERIFY_EXT_LEN) {
            step = (uint8_t)(M1_ID_CAL_VERIFY_EXT_LEN - 1u);
        }
        return deadband_id_cal_clamp_id_ref(s_id_cal_verify_ext_table[step]);
    }
#endif
    if (step >= M1_ID_CAL_AMP_TABLE_LEN) {
        step = (uint8_t)(M1_ID_CAL_AMP_TABLE_LEN - 1u);
    }
    return deadband_id_cal_clamp_id_ref(s_id_cal_amp_table[step]);
}

/** Pass0：低 Id 加长 dwell；Pass1：各档统一 M1_ID_CAL_VERIFY_ID_DWELL_S（默认 0.25 s） */
static float deadband_id_cal_dwell_s_for_ref(float id_ref_a)
{
    const float i_abs = (id_ref_a >= 0.0f) ? id_ref_a : -id_ref_a;

    if (s_id_cal_pass != 0u) {
        return M1_ID_CAL_VERIFY_ID_DWELL_S;
    }
    if (i_abs <= M1_ID_CAL_ID_DWELL_LOW_ID_A) {
        return M1_ID_CAL_ID_DWELL_LOW_S;
    }
    return M1_ID_CAL_ID_DWELL_S;
}

static float deadband_id_cal_dwell_s_for_step(uint8_t step)
{
    return deadband_id_cal_dwell_s_for_ref(deadband_id_cal_ref_from_step(step));
}

static void deadband_id_cal_sweep_arm_bumpless(float id_ref)
{
    s_id_cal_bumpless_id_ref = id_ref;
    s_id_cal_bumpless_arm = 1u;
}

static void deadband_id_cal_sweep_arm_bumpless_iq(float iq_ref)
{
    s_id_cal_iq_bumpless_ref = iq_ref;
    s_id_cal_iq_bumpless_arm = 1u;
}

/** Pass1 大 Id 扫表后清积分，避免 Iq 探路 Ud 饱和锁转子 */
static void deadband_id_cal_prepare_iq_probe_pi(motor_context_t *ctx)
{
    ctx->id_ref = 0.0f;
    ctx->iq_ref = M1_ID_CAL_IQ_PROBE_A;
    foc_pi_reset(&ctx->pi_id);
    foc_pi_reset(&ctx->pi_iq);
    ctx->ud_pi = 0.0f;
    ctx->uq_pi = 0.0f;
    deadband_id_cal_sweep_arm_bumpless(0.0f);
    deadband_id_cal_sweep_arm_bumpless_iq(M1_ID_CAL_IQ_PROBE_A);
}

static void deadband_id_cal_enter_iq_probe_off(motor_context_t *ctx)
{
    s_id_cal_state = M1_ID_CAL_IQ_PROBE_OFF;
    s_id_cal_tick = 0u;
    deadband_id_cal_prepare_iq_probe_pi(ctx);
    deadband_service_apply_profile(DEADBAND_PROFILE_OFF);
    deadband_id_cal_sync_dbg();
}

static void deadband_id_cal_enter_iq_probe_fixed(motor_context_t *ctx)
{
    s_id_cal_state = M1_ID_CAL_IQ_PROBE_FIXED;
    s_id_cal_tick = 0u;
    deadband_id_cal_prepare_iq_probe_pi(ctx);
    deadband_service_apply_profile(DEADBAND_PROFILE_FIXED);
    deadband_id_cal_sync_dbg();
}

static void deadband_id_cal_enter_iq_probe_on(motor_context_t *ctx)
{
    s_id_cal_state = M1_ID_CAL_IQ_PROBE_ON;
    s_id_cal_tick = 0u;
    deadband_id_cal_prepare_iq_probe_pi(ctx);
    deadband_service_apply_profile(DEADBAND_PROFILE_LUT_RUNTIME);
    deadband_id_cal_sync_dbg();
}

static float deadband_id_cal_init_hold_s(void)
{
    if (s_id_cal_pass != 0u) {
        return M1_ID_CAL_VERIFY_INIT_HOLD_S;
    }
    return M1_ID_CAL_INIT_HOLD_S;
}

static void deadband_id_cal_enter_pass0_decay(motor_context_t *ctx)
{
    ctx->id_ref = 0.0f;
    deadband_id_cal_sweep_arm_bumpless(0.0f);
    foc_pi_reset(&ctx->pi_id);
    foc_pi_reset(&ctx->pi_iq);
    ctx->ud_pi = 0.0f;
    ctx->uq_pi = 0.0f;
    s_id_cal_tick = 0u;
    s_id_cal_capture_armed = 0u;
    s_id_cal_capture_pending = 0u;
}

static void deadband_id_cal_pass0_commit_and_arm_verify(motor_context_t *ctx)
{
#if M1_ID_CAL_COMMIT_LUT
    deadband_cal_commit();
#else
    deadband_cal_finish_sweep();
#endif
#if M1_VOFA_LUT_DUMP_ENABLE
    telem_lut_dump_arm();
#endif
    s_id_cal_pass = 1u;
#if M1_ID_CAL_ALIGN_ENABLE && M1_ID_CAL_FIX_THETA_ENABLE
    s_id_cal_state = M1_ID_CAL_ALIGN;
#else
    s_id_cal_state = M1_ID_CAL_INIT_HOLD;
#endif
    s_id_cal_tick = 0u;
    s_id_cal_id_step = 0u;
    ctx->id_ref = 0.0f;
    foc_pi_reset(&ctx->pi_id);
    foc_pi_reset(&ctx->pi_iq);
    ctx->ud_pi = 0.0f;
    ctx->uq_pi = 0.0f;
    deadband_id_cal_sweep_arm_bumpless(0.0f);
    deadband_id_cal_sync_dbg();
}

#if M1_ID_CAL_IQ_PROBE_ENABLE
/** Pass0 decay 结束：commit phase 表，进入 Iq 探路（默认直切 LUT ON） */
static void deadband_id_cal_pass0_commit_enter_iq_probe(motor_context_t *ctx)
{
#if M1_ID_CAL_COMMIT_LUT
    deadband_cal_commit();
#else
    deadband_cal_finish_sweep();
#endif
#if M1_VOFA_LUT_DUMP_ENABLE
    telem_lut_dump_arm();
#endif
    if ((M1_ID_CAL_IQ_PROBE_OFF_S > 0.0f) || (M1_ID_CAL_IQ_PROBE_FIXED_S > 0.0f)) {
        deadband_id_cal_enter_iq_probe_off(ctx);
    } else {
        deadband_id_cal_enter_iq_probe_on(ctx);
    }
}
#endif /* M1_ID_CAL_IQ_PROBE_ENABLE */

static void deadband_id_cal_enter_done(motor_context_t *ctx)
{
    s_id_cal_state = M1_ID_CAL_DONE;
    ctx->id_ref = 0.0f;
    ctx->iq_ref = M1_ID_CAL_IQ_REF_A;
    foc_pi_reset(&ctx->pi_id);
    foc_pi_reset(&ctx->pi_iq);
    ctx->ud_pi = 0.0f;
    ctx->uq_pi = 0.0f;
    deadband_id_cal_sync_dbg();
}

static void deadband_id_cal_pass0_finish(motor_context_t *ctx)
{
#if M1_ID_CAL_COMMIT_LUT
    deadband_cal_commit();
#else
    deadband_cal_finish_sweep();
#endif
#if M1_VOFA_LUT_DUMP_ENABLE
    telem_lut_dump_arm();
#endif
    deadband_id_cal_enter_done(ctx);
}

/** Pass0-only：保留 capture/geo RAM 状态，不 commit LUT、不 VOFA 突发 */
static void deadband_id_cal_pass0_only_finish(motor_context_t *ctx)
{
    deadband_cal_finish_sweep();
    deadband_id_cal_enter_done(ctx);
}

#if M1_ID_CAL_DUAL_ANGLE_ENABLE
static void deadband_id_cal_pass0_leg_done(motor_context_t *ctx)
{
#if M1_ID_CAL_PASS0_ONLY_ENABLE
    if (s_id_cal_pass0_leg + 1u >= deadband_id_cal_pass0_angle_count()) {
        s_id_cal_state = M1_ID_CAL_PASS0_DECAY;
        deadband_id_cal_enter_pass0_decay(ctx);
    } else {
        s_id_cal_state = M1_ID_CAL_PASS0A_DECAY;
        deadband_id_cal_enter_pass0_decay(ctx);
    }
#elif M1_ID_CAL_LUT_VERIFY_SWEEP && M1_ID_CAL_COMMIT_LUT
    if (s_id_cal_pass0_leg + 1u >= deadband_id_cal_pass0_angle_count()) {
        s_id_cal_state = M1_ID_CAL_PASS0_DECAY;
    } else {
        s_id_cal_state = M1_ID_CAL_PASS0A_DECAY;
    }
    deadband_id_cal_enter_pass0_decay(ctx);
#elif M1_ID_CAL_IQ_PROBE_ENABLE && M1_ID_CAL_COMMIT_LUT
    if (s_id_cal_pass0_leg + 1u >= deadband_id_cal_pass0_angle_count()) {
        s_id_cal_state = M1_ID_CAL_PASS0_DECAY;
    } else {
        s_id_cal_state = M1_ID_CAL_PASS0A_DECAY;
    }
    deadband_id_cal_enter_pass0_decay(ctx);
#else
    deadband_id_cal_pass0_finish(ctx);
#endif
}
#endif /* DUAL_ANGLE */

void deadband_id_cal_init(void)
{
#if M1_ID_CAL_ALIGN_ENABLE && M1_ID_CAL_FIX_THETA_ENABLE
    s_id_cal_state = M1_ID_CAL_ALIGN;
#else
    s_id_cal_state = M1_ID_CAL_INIT_HOLD;
#endif
    s_id_cal_tick = 0u;
    s_id_cal_id_step = 0u;
    s_id_cal_bumpless_arm = 0u;
    s_id_cal_bumpless_id_ref = 0.0f;
    s_id_cal_iq_bumpless_arm = 0u;
    s_id_cal_iq_bumpless_ref = 0.0f;
    s_id_cal_capture_pending = 0u;
    s_id_cal_capture_id_ref = 0.0f;
    s_id_cal_capture_armed = 0u;
    s_id_cal_pass = 0u;
#if M1_ID_CAL_DUAL_ANGLE_ENABLE
    s_id_cal_pass0_leg = 0u;
#endif
    deadband_cal_reset();
    deadband_id_cal_sync_dbg();
    deadband_id_cal_sync_open_seq_phase();
}

void deadband_id_cal_tick(motor_context_t *ctx)
{
    const uint32_t init_hold_ticks = deadband_id_cal_ticks_from_s(deadband_id_cal_init_hold_s());
    const uint8_t n_steps = deadband_id_cal_step_count();
    float dwell_s;
    uint32_t dwell_ticks;

#if M1_ID_CAL_IQ_PROBE_ENABLE
    if (deadband_id_cal_in_iq_probe()) {
        ctx->iq_ref = M1_ID_CAL_IQ_PROBE_A;
    } else {
        ctx->iq_ref = M1_ID_CAL_IQ_REF_A;
    }
#else
    ctx->iq_ref = M1_ID_CAL_IQ_REF_A;
#endif

    switch (s_id_cal_state) {
#if M1_ID_CAL_ALIGN_ENABLE && M1_ID_CAL_FIX_THETA_ENABLE
    case M1_ID_CAL_ALIGN:
    case M1_ID_CAL_ALIGN_B: {
#if M1_ID_CAL_DUAL_ANGLE_ENABLE
        const uint32_t align_ticks = deadband_id_cal_align_ticks();
#else
        const uint32_t align_ticks = deadband_id_cal_ticks_from_s(M1_ID_CAL_ALIGN_S);
#endif

        ctx->id_ref = 0.0f;
        s_id_cal_tick++;
        if (align_ticks == 0u || s_id_cal_tick >= align_ticks) {
#if M1_ID_CAL_DUAL_ANGLE_ENABLE && !M1_ID_CAL_MULTI_ANGLE_ENABLE
            if (s_id_cal_state == M1_ID_CAL_ALIGN_B) {
                s_id_cal_pass0_leg = 1u;
            }
#endif
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

    case M1_ID_CAL_INIT_HOLD:
        ctx->id_ref = 0.0f;
        s_id_cal_tick++;
        if (s_id_cal_tick >= init_hold_ticks) {
            s_id_cal_state = M1_ID_CAL_ID_STEP;
            s_id_cal_tick = 0u;
            s_id_cal_id_step = 0u;
            ctx->id_ref = deadband_id_cal_ref_from_step(s_id_cal_id_step);
            deadband_id_cal_sweep_arm_bumpless(ctx->id_ref);
        }
        break;

#if M1_ID_CAL_PASS0_DECAY_ENABLE
    case M1_ID_CAL_PASS0A_DECAY: {
        const uint32_t pass0_decay_ticks =
            deadband_id_cal_ticks_from_s(M1_ID_CAL_PASS0_DECAY_S);

        ctx->id_ref = 0.0f;
        s_id_cal_tick++;
        if (s_id_cal_tick >= pass0_decay_ticks) {
#if M1_ID_CAL_DUAL_ANGLE_ENABLE
#if M1_ID_CAL_MULTI_ANGLE_ENABLE
            s_id_cal_pass0_leg++;
            s_id_cal_state = M1_ID_CAL_ALIGN;
#else
            s_id_cal_state = M1_ID_CAL_ALIGN_B;
#endif
#else
            deadband_id_cal_pass0_commit_and_arm_verify(ctx);
#endif
            s_id_cal_tick = 0u;
            foc_pi_reset(&ctx->pi_id);
            foc_pi_reset(&ctx->pi_iq);
            ctx->ud_pi = 0.0f;
            ctx->uq_pi = 0.0f;
        }
        break;
    }

    case M1_ID_CAL_PASS0_DECAY: {
        const uint32_t pass0_decay_ticks =
            deadband_id_cal_ticks_from_s(M1_ID_CAL_PASS0_DECAY_S);

        ctx->id_ref = 0.0f;
        s_id_cal_tick++;
        if (s_id_cal_tick >= pass0_decay_ticks) {
#if M1_ID_CAL_PASS0_ONLY_ENABLE
            deadband_id_cal_pass0_only_finish(ctx);
#elif M1_ID_CAL_IQ_PROBE_ENABLE && M1_ID_CAL_COMMIT_LUT && !M1_ID_CAL_LUT_VERIFY_SWEEP
            deadband_id_cal_pass0_commit_enter_iq_probe(ctx);
#elif M1_ID_CAL_LUT_VERIFY_SWEEP && M1_ID_CAL_COMMIT_LUT
            deadband_id_cal_pass0_commit_and_arm_verify(ctx);
#else
            deadband_id_cal_pass0_finish(ctx);
#endif
        }
        break;
    }
#endif /* M1_ID_CAL_PASS0_DECAY_ENABLE */

    case M1_ID_CAL_ID_STEP:
        ctx->id_ref = deadband_id_cal_ref_from_step(s_id_cal_id_step);
        dwell_s = deadband_id_cal_dwell_s_for_step(s_id_cal_id_step);
        dwell_ticks = deadband_id_cal_ticks_from_s(dwell_s);
        s_id_cal_tick++;
        if (s_id_cal_pass == 0u &&
            dwell_ticks >= 5u &&
            s_id_cal_tick >= (dwell_ticks - dwell_ticks / 5u) &&
            !s_id_cal_capture_armed) {
            s_id_cal_capture_id_ref = deadband_id_cal_ref_from_step(s_id_cal_id_step);
            s_id_cal_capture_pending = 1u;
            s_id_cal_capture_armed = 1u;
        }
        if (s_id_cal_tick >= dwell_ticks) {
            s_id_cal_tick = 0u;
            s_id_cal_id_step++;
            s_id_cal_capture_armed = 0u;
            if (s_id_cal_id_step >= n_steps) {
                if (s_id_cal_pass == 0u) {
#if M1_ID_CAL_DUAL_ANGLE_ENABLE
                    deadband_id_cal_pass0_leg_done(ctx);
#elif M1_ID_CAL_PASS0_DECAY_ENABLE
                    s_id_cal_state = M1_ID_CAL_PASS0_DECAY;
                    deadband_id_cal_enter_pass0_decay(ctx);
#else
                    deadband_id_cal_pass0_finish(ctx);
#endif
                } else {
#if M1_ID_CAL_IQ_PROBE_ENABLE
                    deadband_id_cal_enter_iq_probe_off(ctx);
#else
                    deadband_id_cal_enter_done(ctx);
#endif
                }
            } else {
                ctx->id_ref = deadband_id_cal_ref_from_step(s_id_cal_id_step);
                deadband_id_cal_sweep_arm_bumpless(ctx->id_ref);
            }
        }
        break;

#if M1_ID_CAL_IQ_PROBE_ENABLE
    case M1_ID_CAL_IQ_PROBE_OFF:
        ctx->id_ref = 0.0f;
        s_id_cal_tick++;
        if (s_id_cal_tick >= deadband_id_cal_ticks_from_s(M1_ID_CAL_IQ_PROBE_OFF_S)) {
            if (M1_ID_CAL_IQ_PROBE_FIXED_S > 0.0f) {
                deadband_id_cal_enter_iq_probe_fixed(ctx);
            } else {
                deadband_id_cal_enter_iq_probe_on(ctx);
            }
        }
        break;

    case M1_ID_CAL_IQ_PROBE_FIXED:
        ctx->id_ref = 0.0f;
        s_id_cal_tick++;
        if (s_id_cal_tick >= deadband_id_cal_ticks_from_s(M1_ID_CAL_IQ_PROBE_FIXED_S)) {
            deadband_id_cal_enter_iq_probe_on(ctx);
        }
        break;

    case M1_ID_CAL_IQ_PROBE_ON:
        ctx->id_ref = 0.0f;
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
    return !deadband_id_cal_in_iq_probe();
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
    return (s_id_cal_pass == 0u && s_id_cal_capture_pending) ? 1u : 0u;
}

float deadband_id_cal_capture_id_ref(void)
{
    return s_id_cal_capture_id_ref;
}

void deadband_id_cal_clear_capture_pending(void)
{
    s_id_cal_capture_pending = 0u;
}

#else /* !M1_ID_LOCK_CAL_SWEEP */

void deadband_id_cal_init(void) {}
void deadband_id_cal_tick(motor_context_t *ctx) { (void)ctx; }
uint8_t deadband_id_cal_is_running(void) { return 0u; }
uint8_t deadband_id_cal_is_done(void) { return 0u; }
uint8_t deadband_id_cal_use_fix_theta(void) { return 0u; }
float deadband_id_cal_target_theta(void) { return 0.0f; }
uint8_t deadband_id_cal_use_align_ud(void) { return 0u; }
float deadband_id_cal_align_ud_v(void) { return 0.0f; }
uint8_t deadband_id_cal_in_iq_probe(void) { return 0u; }
uint8_t deadband_id_cal_use_cal_pi_limits(void) { return 0u; }
float deadband_id_cal_clamp_id_ref(float ref) { return ref; }
uint8_t deadband_id_cal_bumpless_arm(void) { return 0u; }
float deadband_id_cal_bumpless_id_ref(void) { return 0.0f; }
void deadband_id_cal_consume_bumpless_arm(void) {}
uint8_t deadband_id_cal_iq_bumpless_arm(void) { return 0u; }
float deadband_id_cal_iq_bumpless_ref(void) { return 0.0f; }
void deadband_id_cal_consume_iq_bumpless_arm(void) {}
uint8_t deadband_id_cal_should_capture(void) { return 0u; }
float deadband_id_cal_capture_id_ref(void) { return 0.0f; }
uint8_t deadband_id_cal_capture_append_dlut(void) { return 0u; }
void deadband_id_cal_clear_capture_pending(void) {}
void deadband_id_cal_sync_dbg(void) {}

#endif /* M1_ID_LOCK_CAL_SWEEP */
