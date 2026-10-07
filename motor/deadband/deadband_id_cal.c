/**
 * @file deadband_id_cal.c
 * @date 2026-10-06
 * @brief Id 锁轴扫表编排实现。

 *
 * 节拍限制见 deadband_id_cal.h 文件头。
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
 */

#include "deadband_id_cal.h"

#include "deadband_cal.h"
#include "deadband_service.h"
#include "foc_pi.h"
#include "dbg_monitor.h"
#include "motor_cfg.h"
#include "motor_params_m1.h"
#include "rs_ident.h"
#include "ld_lq_ident.h"
#include "telem_ident_dump.h"
#include "telem_lut_dump.h"
#if M1_ID_CAL_RS_LD_LQ_ONLY_ENABLE
#include "factory_nvm.h"
#if M1_DEADBAND_LUT_BAKED_ENABLE
#include "deadband_lut_baked_m1.h"
#endif
#endif

#if M1_ID_LOCK_CAL_SWEEP

float deadband_id_cal_clamp_id_ref(float ref);

/**
 * Pass0 Id 扫表档位列（长度须 = M1_ID_CAL_AMP_TABLE_LEN）：
 *   138：低 I 0.05～0.40 ×4 + 中高至 3 A（DUAL_FULL）
 *    30：0.05～1.50 A @50 mA/档（IDENT 单角快扫）
 */
#if (M1_ID_CAL_AMP_TABLE_LEN == 30u)
static const float s_id_cal_amp_table[M1_ID_CAL_AMP_TABLE_LEN] = {
    0.0500f, 0.1000f, 0.1500f, 0.2000f, 0.2500f, 0.3000f, 0.3500f, 0.4000f,
    0.4500f, 0.5000f, 0.5500f, 0.6000f, 0.6500f, 0.7000f, 0.7500f, 0.8000f,
    0.8500f, 0.9000f, 0.9500f, 1.0000f, 1.0500f, 1.1000f, 1.1500f, 1.2000f,
    1.2500f, 1.3000f, 1.3500f, 1.4000f, 1.4500f, 1.5000f,
};
#elif (M1_ID_CAL_AMP_TABLE_LEN == 138u)
static const float s_id_cal_amp_table[M1_ID_CAL_AMP_TABLE_LEN] = {
    0.0500f, 0.0531f, 0.0563f, 0.0594f, 0.0625f, 0.0656f, 0.0688f, 0.0719f,
    0.0750f, 0.0781f, 0.0813f, 0.0844f, 0.0875f, 0.0906f, 0.0938f, 0.0969f,
    0.1000f, 0.1031f, 0.1063f, 0.1094f, 0.1125f, 0.1156f, 0.1188f, 0.1219f,
    0.1250f, 0.1281f, 0.1313f, 0.1344f, 0.1375f, 0.1406f, 0.1438f, 0.1469f,
    0.1500f, 0.1531f, 0.1563f, 0.1594f, 0.1625f, 0.1656f, 0.1688f, 0.1719f,
    0.1750f, 0.1781f, 0.1813f, 0.1844f, 0.1875f, 0.1906f, 0.1938f, 0.1969f,
    0.2000f, 0.2031f, 0.2063f, 0.2094f, 0.2125f, 0.2156f, 0.2188f, 0.2219f,
    0.2250f, 0.2281f, 0.2313f, 0.2344f, 0.2375f, 0.2406f, 0.2438f, 0.2469f,
    0.2500f, 0.2531f, 0.2563f, 0.2594f, 0.2625f, 0.2656f, 0.2688f, 0.2719f,
    0.2750f, 0.2781f, 0.2813f, 0.2844f, 0.2875f, 0.2906f, 0.2938f, 0.2969f,
    0.3000f, 0.3031f, 0.3063f, 0.3094f, 0.3125f, 0.3156f, 0.3188f, 0.3219f,
    0.3250f, 0.3281f, 0.3313f, 0.3344f, 0.3375f, 0.3406f, 0.3438f, 0.3469f,
    0.3500f, 0.3531f, 0.3563f, 0.3594f, 0.3625f, 0.3656f, 0.3688f, 0.3719f,
    0.3750f, 0.3781f, 0.3813f, 0.3844f, 0.3875f, 0.3906f, 0.3938f, 0.3969f,
    0.4000f, 0.4500f, 0.5250f, 0.6000f, 0.6750f, 0.7500f, 0.8250f, 0.9000f,
    0.9750f, 1.0500f, 1.1250f, 1.2000f, 1.2750f, 1.3500f, 1.5000f, 1.5500f,
    1.6833f, 1.8167f, 1.9500f, 2.0833f, 2.2167f, 2.3500f, 2.4833f, 2.6167f,
    2.7500f, 3.0000f,
};
#else
#error "Unsupported M1_ID_CAL_AMP_TABLE_LEN — use 30 (IDENT) or 138 (DUAL_FULL)"
#endif

#if M1_ID_CAL_LUT_VERIFY_SWEEP && M1_ID_CAL_COMMIT_LUT && (M1_ID_CAL_VERIFY_EXT_LEN > 0u)
/** Pass1 续扫：Pass0 末档之后追加（EXT_LEN=0 时 Pass1 复用 Pass0 同表） */
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
    M1_ID_CAL_RS_IDENT,
    M1_ID_CAL_LD_LQ_PRE_DECAY,
    M1_ID_CAL_LD_LQ_ALIGN,
    M1_ID_CAL_LD_LQ_IDENT,
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
#if M1_LD_LQ_IDENT_ENABLE && M1_DEADBAND_FLOW_LD_LQ_AFTER_OPEN_UD
/** Pass0→Ud 阶梯后链式 VASI：INIT_HOLD 走 fixed-Rs Ld/Lq，且不 deadband_cal_reset */
static uint8_t s_id_cal_ld_lq_chain_pending;
#endif
#if M1_ID_CAL_DUAL_ANGLE_ENABLE
/** Pass0 角序号：双角 0=30° 1=0°；MULTI 时 0..N-1 见 s_id_cal_pass0_theta_rad[] */
static uint8_t s_id_cal_pass0_leg;
#if M1_ID_CAL_MULTI_ANGLE_ENABLE
#if (M1_ID_CAL_PASS0_ANGLE_COUNT == 3)
/** 三角 Pass0：30/150/270°，各 +120° 强相轮换 */
static const float s_id_cal_pass0_theta_rad[3] = {
    M1_ID_CAL_THETA_PASS0_A_RAD,
    M1_ID_CAL_THETA_PASS0_150_RAD,
    M1_ID_CAL_THETA_PASS0_270_RAD,
};
#else
static const float s_id_cal_pass0_theta_rad[M1_ID_CAL_PASS0_ANGLE_COUNT] = {
    M1_ID_CAL_THETA_PASS0_0_RAD,
    M1_ID_CAL_THETA_PASS0_A_RAD,
    M1_ID_CAL_THETA_PASS0_60_RAD,
    M1_ID_CAL_THETA_PASS0_90_RAD,
    M1_ID_CAL_THETA_PASS0_120_RAD,
};
#endif
#endif
#endif

#if M1_LD_LQ_MULTI_ANGLE_ENABLE
static uint8_t s_ld_lq_angle_leg;
#if (M1_LD_LQ_IDENT_ANGLE_COUNT == 3u)
/** 与 Pass0 三角一致：30° / 150° / 270° */
static const float s_ld_lq_theta_rad[3] = {
    M1_LD_LQ_THETA0_EL_RAD,
    M1_LD_LQ_THETA1_EL_RAD,
    M1_LD_LQ_THETA2_EL_RAD,
};
#else
static const float s_ld_lq_theta_rad[M1_LD_LQ_IDENT_ANGLE_COUNT] = {
    M1_LD_LQ_THETA0_EL_RAD,
    M1_LD_LQ_THETA1_EL_RAD,
    M1_LD_LQ_THETA2_EL_RAD,
};
#endif
#endif /* M1_LD_LQ_MULTI_ANGLE_ENABLE */

#if M1_LD_LQ_IDENT_ENABLE
static float s_ld_lq_rs_cached;
#if M1_RS_L_IDENT_DUAL_LUT_ROUND_ENABLE
/** 0=OFF 轮 Rs+VASI，1=LUT 轮 */
static uint8_t s_rs_l_lut_round;
#endif
#endif /* M1_LD_LQ_IDENT_ENABLE */

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

#if M1_RS_IDENT_ENABLE
uint8_t deadband_id_cal_in_rs_ident(void)
{
    return (s_id_cal_state == M1_ID_CAL_RS_IDENT) ? 1u : 0u;
}
#endif

#if M1_LD_LQ_IDENT_ENABLE
uint8_t deadband_id_cal_in_ld_lq_ident(void)
{
    return (s_id_cal_state == M1_ID_CAL_LD_LQ_IDENT) ? 1u : 0u;
}

uint8_t deadband_id_cal_in_ld_lq_pre_decay(void)
{
    return (s_id_cal_state == M1_ID_CAL_LD_LQ_PRE_DECAY) ? 1u : 0u;
}

#if M1_LD_LQ_MULTI_ANGLE_ENABLE
uint8_t deadband_id_cal_in_ld_lq_sweep(void)
{
    return (s_id_cal_state == M1_ID_CAL_LD_LQ_ALIGN) ||
           (s_id_cal_state == M1_ID_CAL_LD_LQ_IDENT);
}
#endif

uint8_t deadband_id_cal_use_ld_lq_align_ud(void)
{
    return (s_id_cal_state == M1_ID_CAL_LD_LQ_ALIGN) ? 1u : 0u;
}

static uint8_t deadband_id_cal_ld_lq_vasi_open_seq(uint8_t leg)
{
    return (uint8_t)(M1_LD_LQ_OPEN_SEQ_VASI_BASE + leg * M1_LD_LQ_OPEN_SEQ_VASI_STRIDE);
}

static void deadband_id_cal_sweep_arm_bumpless(float id_ref);
static void deadband_id_cal_sync_open_seq_phase(void);
static void deadband_id_cal_sync_lut_round_dbg(void);

#if M1_RS_L_IDENT_DUAL_LUT_ROUND_ENABLE
/** 启动一轮 Rs ramp（round 0=OFF，1=LUT runtime abc） */
static void deadband_id_cal_begin_rs_l_lut_round(motor_context_t *ctx, uint8_t lut_round)
{
    s_rs_l_lut_round = (lut_round != 0u) ? 1u : 0u;
    deadband_id_cal_sync_lut_round_dbg();
    if (s_rs_l_lut_round != 0u) {
        deadband_service_apply_profile(DEADBAND_PROFILE_LUT_RUNTIME);
    } else {
        deadband_service_apply_profile(DEADBAND_PROFILE_OFF);
    }
    rs_ident_arm();
    s_id_cal_state = M1_ID_CAL_RS_IDENT;
    s_id_cal_tick = 0u;
    ctx->id_ref = 0.0f;
    ctx->iq_ref = 0.0f;
    foc_pi_reset(&ctx->pi_id);
    foc_pi_reset(&ctx->pi_iq);
    ctx->ud_pi = 0.0f;
    ctx->uq_pi = 0.0f;
    deadband_id_cal_sweep_arm_bumpless(0.0f);
    deadband_id_cal_sync_dbg();
    deadband_id_cal_sync_open_seq_phase();
}
#endif /* M1_RS_L_IDENT_DUAL_LUT_ROUND_ENABLE */

#endif /* M1_LD_LQ_IDENT_ENABLE */

#if M1_RS_L_IDENT_DUAL_LUT_ROUND_ENABLE && M1_LD_LQ_IDENT_ENABLE
static uint8_t deadband_id_cal_lut_round_open_seq_offset(void)
{
    return (uint8_t)(s_rs_l_lut_round * M1_RS_L_IDENT_LUT_ROUND_OPEN_SEQ_OFFSET);
}

static void deadband_id_cal_sync_lut_round_dbg(void)
{
    dbg.rs_l_ident_lut_round = (float)s_rs_l_lut_round;
}
#else
static uint8_t deadband_id_cal_lut_round_open_seq_offset(void)
{
    return 0u;
}

static void deadband_id_cal_sync_lut_round_dbg(void)
{
    dbg.rs_l_ident_lut_round = 0.0f;
}
#endif /* M1_RS_L_IDENT_DUAL_LUT_ROUND_ENABLE && M1_LD_LQ_IDENT_ENABLE */

#if M1_ID_CAL_FIX_THETA_ENABLE
uint8_t deadband_id_cal_use_fix_theta(void)
{
#if M1_VOFA_IDENT_DUMP_ENABLE
    /* ident 突发期间保持 Park@30°，避免 Ud=0 时转子滑移 */
    if (s_id_cal_state == M1_ID_CAL_DONE && telem_ident_dump_is_busy()) {
        return 1u;
    }
#endif
    return deadband_id_cal_is_running() && !deadband_id_cal_in_iq_probe();
}
#endif

#if M1_ID_CAL_ALIGN_ENABLE && M1_ID_CAL_FIX_THETA_ENABLE && M1_VOFA_IDENT_DUMP_ENABLE
uint8_t deadband_id_cal_use_post_ident_hold_ud(void)
{
    return (s_id_cal_state == M1_ID_CAL_DONE && telem_ident_dump_is_busy()) ? 1u : 0u;
}
#endif

static void deadband_id_cal_sync_open_seq_phase(void)
{
    if (s_id_cal_state == M1_ID_CAL_DONE) {
#if M1_LD_LQ_IDENT_ENABLE
        dbg.open_seq_phase = (uint8_t)(ld_lq_ident_open_seq_phase() +
                                       deadband_id_cal_lut_round_open_seq_offset());
#elif M1_RS_IDENT_ENABLE
        dbg.open_seq_phase = rs_ident_open_seq_phase();
#else
        dbg.open_seq_phase = 9u;
#endif
    } else if (s_id_cal_state == M1_ID_CAL_IQ_PROBE_OFF) {
        dbg.open_seq_phase = 50u;
    } else if (s_id_cal_state == M1_ID_CAL_IQ_PROBE_FIXED) {
        dbg.open_seq_phase = 53u;
    } else if (s_id_cal_state == M1_ID_CAL_IQ_PROBE_ON) {
        dbg.open_seq_phase = 51u;
#if M1_RS_IDENT_ENABLE
    } else if (s_id_cal_state == M1_ID_CAL_RS_IDENT) {
        dbg.open_seq_phase = (uint8_t)(rs_ident_open_seq_phase() +
                                       deadband_id_cal_lut_round_open_seq_offset());
#endif
#if M1_LD_LQ_IDENT_ENABLE
    } else if (s_id_cal_state == M1_ID_CAL_LD_LQ_PRE_DECAY) {
        dbg.open_seq_phase = (uint8_t)(M1_LD_LQ_OPEN_SEQ_PRE_DECAY +
                                       deadband_id_cal_lut_round_open_seq_offset());
    } else if (s_id_cal_state == M1_ID_CAL_LD_LQ_ALIGN) {
#if M1_LD_LQ_MULTI_ANGLE_ENABLE
        dbg.open_seq_phase = (uint8_t)(160u + s_ld_lq_angle_leg);
#else
        dbg.open_seq_phase = 160u;
#endif
    } else if (s_id_cal_state == M1_ID_CAL_LD_LQ_IDENT) {
#if M1_LD_LQ_MULTI_ANGLE_ENABLE
        dbg.open_seq_phase = deadband_id_cal_ld_lq_vasi_open_seq(s_ld_lq_angle_leg);
#else
        dbg.open_seq_phase = (uint8_t)(ld_lq_ident_open_seq_phase() +
                                       deadband_id_cal_lut_round_open_seq_offset());
#endif
#endif
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

static void deadband_id_cal_enter_done(motor_context_t *ctx);

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
#endif /* DUAL_ANGLE + FIX_THETA pass0 helpers */

#if M1_ID_CAL_FIX_THETA_ENABLE
float deadband_id_cal_target_theta(void)
{
#if M1_LD_LQ_MULTI_ANGLE_ENABLE
    if (s_id_cal_state == M1_ID_CAL_LD_LQ_ALIGN ||
        s_id_cal_state == M1_ID_CAL_LD_LQ_IDENT) {
        uint8_t leg = s_ld_lq_angle_leg;

        if (leg >= M1_LD_LQ_IDENT_ANGLE_COUNT) {
            leg = (uint8_t)(M1_LD_LQ_IDENT_ANGLE_COUNT - 1u);
        }
        return s_ld_lq_theta_rad[leg];
    }
#endif
#if M1_ID_CAL_DUAL_ANGLE_ENABLE
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
#else
    return M1_ID_CAL_THETA_EL_RAD;
#endif
}
#endif /* M1_ID_CAL_FIX_THETA_ENABLE */

#if M1_ID_CAL_DUAL_ANGLE_ENABLE && M1_ID_CAL_FIX_THETA_ENABLE
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
#if M1_ID_CAL_VERIFY_EXT_LEN > 0u
        step = (uint8_t)(step - M1_ID_CAL_AMP_TABLE_LEN);
        if (step >= M1_ID_CAL_VERIFY_EXT_LEN) {
            step = (uint8_t)(M1_ID_CAL_VERIFY_EXT_LEN - 1u);
        }
        return deadband_id_cal_clamp_id_ref(s_id_cal_verify_ext_table[step]);
#else
        return deadband_id_cal_clamp_id_ref(
            s_id_cal_amp_table[M1_ID_CAL_AMP_TABLE_LEN - 1u]);
#endif
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

#if M1_RS_IDENT_ENABLE
/** 死区 OFF 启动 Rs 斜坡（单轮签收用） */
static void deadband_id_cal_begin_rs_ident_off(motor_context_t *ctx)
{
    deadband_service_apply_profile(DEADBAND_PROFILE_OFF);
    deadband_id_cal_sync_lut_round_dbg();
    rs_ident_arm();
    s_id_cal_state = M1_ID_CAL_RS_IDENT;
    s_id_cal_tick = 0u;
    ctx->id_ref = 0.0f;
    ctx->iq_ref = 0.0f;
    foc_pi_reset(&ctx->pi_id);
    foc_pi_reset(&ctx->pi_iq);
    ctx->ud_pi = 0.0f;
    ctx->uq_pi = 0.0f;
    deadband_id_cal_sweep_arm_bumpless(0.0f);
    deadband_id_cal_sync_dbg();
    deadband_id_cal_sync_open_seq_phase();
}

/** Pass0 decay 结束：commit → 单轮 OFF Rs+VASI（或双轮时 round0 OFF → round1 LUT） */
static void deadband_id_cal_pass0_commit_enter_rs_ident(motor_context_t *ctx)
{
#if M1_ID_CAL_COMMIT_LUT
    deadband_cal_commit();
#else
    deadband_cal_finish_sweep();
#endif
#if M1_VOFA_LUT_DUMP_ENABLE
    telem_lut_dump_arm();
#endif
#if M1_RS_L_IDENT_DUAL_LUT_ROUND_ENABLE
    deadband_id_cal_begin_rs_l_lut_round(ctx, 0u);
#else
    deadband_id_cal_begin_rs_ident_off(ctx);
#endif
}
#endif /* M1_RS_IDENT_ENABLE */

#if M1_LD_LQ_IDENT_ENABLE
static void deadband_id_cal_start_ld_lq_ident(motor_context_t *ctx, float rs_ohm);

static void deadband_id_cal_enter_ld_lq_pre_decay(motor_context_t *ctx)
{
    s_id_cal_state = M1_ID_CAL_LD_LQ_PRE_DECAY;
    s_id_cal_tick = 0u;
    ctx->id_ref = 0.0f;
    ctx->iq_ref = 0.0f;
    foc_pi_reset(&ctx->pi_id);
    foc_pi_reset(&ctx->pi_iq);
    ctx->ud_pi = 0.0f;
    ctx->uq_pi = 0.0f;
    deadband_id_cal_sweep_arm_bumpless(0.0f);
    deadband_id_cal_sweep_arm_bumpless_iq(0.0f);
    deadband_id_cal_sync_dbg();
}

static void deadband_id_cal_leave_ld_lq_pre_decay(motor_context_t *ctx)
{
#if M1_LD_LQ_MULTI_ANGLE_ENABLE
    s_id_cal_state = M1_ID_CAL_LD_LQ_ALIGN;
    s_id_cal_tick = 0u;
    ctx->id_ref = 0.0f;
    ctx->iq_ref = 0.0f;
    foc_pi_reset(&ctx->pi_id);
    foc_pi_reset(&ctx->pi_iq);
    ctx->ud_pi = 0.0f;
    ctx->uq_pi = 0.0f;
    deadband_id_cal_sweep_arm_bumpless(0.0f);
    deadband_id_cal_sweep_arm_bumpless_iq(0.0f);
    deadband_id_cal_sync_dbg();
#else
    deadband_id_cal_start_ld_lq_ident(ctx, s_ld_lq_rs_cached);
#endif
}

static void deadband_id_cal_cache_rs_for_ld_lq(void)
{
    rs_ident_result_t rs;

    s_ld_lq_rs_cached = g_m1_motor_cfg.rs_ohm;
    rs_ident_get_result(&rs);
    if (rs.ok != 0u && rs.rs_ohm > 0.0f) {
        s_ld_lq_rs_cached = rs.rs_ohm;
    }
}

static void deadband_id_cal_start_ld_lq_ident(motor_context_t *ctx, float rs_ohm)
{
    float id0 = 0.0f;
    float iq0 = 0.0f;

#if M1_LD_LQ_MULTI_ANGLE_ENABLE
    const float theta_el = s_ld_lq_theta_rad[s_ld_lq_angle_leg];

    ld_lq_ident_set_angle_leg(s_ld_lq_angle_leg, theta_el);
#else
    (void)0;
#endif
    ld_lq_ident_arm(rs_ohm);
    ld_lq_ident_first_grid_bias(&id0, &iq0);
    s_id_cal_state = M1_ID_CAL_LD_LQ_IDENT;
    s_id_cal_tick = 0u;
    ctx->id_ref = 0.0f;
    ctx->iq_ref = 0.0f;
    foc_pi_reset(&ctx->pi_id);
    foc_pi_reset(&ctx->pi_iq);
    ctx->ud_pi = 0.0f;
    ctx->uq_pi = 0.0f;
    deadband_id_cal_sweep_arm_bumpless(id0);
    deadband_id_cal_sweep_arm_bumpless_iq(iq0);
    deadband_id_cal_sync_dbg();
}

/** Rs 辨识结束 → decay → VASI 9 点 grid Ld/Lq */
static void deadband_id_cal_begin_ld_lq_after_rs(motor_context_t *ctx)
{
    deadband_id_cal_cache_rs_for_ld_lq();
#if M1_LD_LQ_MULTI_ANGLE_ENABLE
    s_ld_lq_angle_leg = 0u;
#endif
    deadband_id_cal_enter_ld_lq_pre_decay(ctx);
}

#if M1_RS_IDENT_USE_FIXED_NOMINAL
/** 跳过 Rs ramp：死区 OFF，固定 M1_RS_OHM → PRE_DECAY → VASI */
static void deadband_id_cal_begin_ld_lq_with_fixed_rs(motor_context_t *ctx)
{
    deadband_service_apply_profile(DEADBAND_PROFILE_OFF);
    deadband_id_cal_sync_lut_round_dbg();
    s_ld_lq_rs_cached = g_m1_motor_cfg.rs_ohm;
#if M1_LD_LQ_MULTI_ANGLE_ENABLE
    s_ld_lq_angle_leg = 0u;
#endif
    ctx->id_ref = 0.0f;
    ctx->iq_ref = 0.0f;
    foc_pi_reset(&ctx->pi_id);
    foc_pi_reset(&ctx->pi_iq);
    ctx->ud_pi = 0.0f;
    ctx->uq_pi = 0.0f;
    deadband_id_cal_sweep_arm_bumpless(0.0f);
    deadband_id_cal_sweep_arm_bumpless_iq(0.0f);
    deadband_id_cal_enter_ld_lq_pre_decay(ctx);
    deadband_id_cal_sync_dbg();
    deadband_id_cal_sync_open_seq_phase();
}
#endif /* M1_RS_IDENT_USE_FIXED_NOMINAL */

static void deadband_id_cal_ld_lq_leg_done(motor_context_t *ctx)
{
    ld_lq_ident_sync_dbg();
#if M1_LD_LQ_MULTI_ANGLE_ENABLE
    ld_lq_ident_commit_leg(s_ld_lq_angle_leg);
    s_ld_lq_angle_leg++;
    if (s_ld_lq_angle_leg < M1_LD_LQ_IDENT_ANGLE_COUNT) {
        deadband_id_cal_enter_ld_lq_pre_decay(ctx);
    } else {
        deadband_id_cal_enter_done(ctx);
    }
#else
#if M1_RS_L_IDENT_DUAL_LUT_ROUND_ENABLE
    if (s_rs_l_lut_round == 0u) {
        deadband_id_cal_begin_rs_l_lut_round(ctx, 1u);
    } else {
        deadband_id_cal_enter_done(ctx);
    }
#else
    deadband_id_cal_enter_done(ctx);
#endif
#endif
}
#endif /* M1_LD_LQ_IDENT_ENABLE */

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
#if (M1_ID_CAL_COMMIT_LUT || M1_ID_CAL_RS_LD_LQ_ONLY_ENABLE) && \
    !M1_RS_L_IDENT_DUAL_LUT_ROUND_ENABLE
    /* 辨识结束：恢复 runtime LUT 供后续电流环 */
    deadband_service_apply_profile(DEADBAND_PROFILE_LUT_RUNTIME);
#endif
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

static void deadband_id_cal_pass0_finish(motor_context_t *ctx)
{
#if M1_ID_CAL_COMMIT_LUT
    deadband_cal_commit();
#else
    deadband_cal_finish_sweep();
#endif
#if M1_VOFA_LUT_DUMP_ENABLE && !M1_OPEN_UD_AFTER_ID_CAL
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
#if M1_RS_IDENT_ENABLE
    rs_ident_init();
#endif
#if M1_LD_LQ_IDENT_ENABLE
    ld_lq_ident_init();
#if M1_RS_L_IDENT_DUAL_LUT_ROUND_ENABLE
    s_rs_l_lut_round = 0u;
#endif
#endif
    deadband_id_cal_sync_lut_round_dbg();
    deadband_id_cal_sync_dbg();
    deadband_id_cal_sync_open_seq_phase();
}

#if M1_ID_CAL_IQ_PROBE_ONLY_ENABLE
void deadband_id_cal_boot_iq_probe_only(motor_context_t *ctx)
{
    if (ctx == NULL) {
        return;
    }
    deadband_id_cal_enter_iq_probe_off(ctx);
}
#endif

#if M1_ID_CAL_RS_LD_LQ_ONLY_ENABLE
/** 跳过 Pass0：NVM 恢复 LUT；ALIGN→HOLD→VASI（Rs 固定 M1_RS_OHM）由 init 状态机接管 */
void deadband_id_cal_boot_rs_ld_lq_only(motor_context_t *ctx)
{
    if (ctx == NULL) {
        return;
    }
#if M1_DEADBAND_LUT_BAKED_ENABLE
    (void)deadband_lut_baked_m1_apply();
#elif M1_DEADBAND_NVM_ON_BOOT
    (void)factory_nvm_apply_deadband();
#endif
    deadband_service_apply_profile(DEADBAND_PROFILE_OFF);
}
#endif

#if M1_LD_LQ_IDENT_ENABLE && M1_DEADBAND_FLOW_LD_LQ_AFTER_OPEN_UD
void deadband_id_cal_reinit_ld_lq_chain(motor_context_t *ctx)
{
    if (ctx == NULL) {
        return;
    }

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
    ld_lq_ident_init();
#if M1_RS_L_IDENT_DUAL_LUT_ROUND_ENABLE
    s_ld_lq_lut_round = 0u;
#endif
    s_id_cal_ld_lq_chain_pending = 1u;
    ctx->mode = M1_CTRL_CURRENT_LOOP;
    ctx->id_ref = 0.0f;
    ctx->iq_ref = M1_ID_CAL_IQ_REF_A;
    ctx->uq_open = 0.0f;
    ctx->ud_open = 0.0f;
    foc_pi_reset(&ctx->pi_id);
    foc_pi_reset(&ctx->pi_iq);
    ctx->ud_pi = 0.0f;
    ctx->uq_pi = 0.0f;
    deadband_service_apply_profile(DEADBAND_PROFILE_OFF);
    deadband_id_cal_sync_dbg();
}
#endif

void deadband_id_cal_tick(motor_context_t *ctx)
{
    const uint32_t init_hold_ticks = deadband_id_cal_ticks_from_s(deadband_id_cal_init_hold_s());
    const uint8_t n_steps = deadband_id_cal_step_count();
    float dwell_s;
    uint32_t dwell_ticks;

#if M1_ID_CAL_IQ_PROBE_ENABLE
    if (deadband_id_cal_in_iq_probe()) {
        ctx->iq_ref = M1_ID_CAL_IQ_PROBE_A;
    } else if (!deadband_id_cal_in_ld_lq_ident() && !deadband_id_cal_in_rs_ident() &&
               !deadband_id_cal_in_ld_lq_pre_decay()) {
        ctx->iq_ref = M1_ID_CAL_IQ_REF_A;
    }
#else
    if (!deadband_id_cal_in_ld_lq_ident() && !deadband_id_cal_in_rs_ident() &&
        !deadband_id_cal_in_ld_lq_pre_decay()) {
        ctx->iq_ref = M1_ID_CAL_IQ_REF_A;
    }
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
#if M1_ID_CAL_RS_LD_LQ_ONLY_ENABLE
#if M1_RS_IDENT_USE_FIXED_NOMINAL
            deadband_id_cal_begin_ld_lq_with_fixed_rs(ctx);
#elif M1_RS_IDENT_ENABLE
            deadband_id_cal_begin_rs_ident_off(ctx);
#else
#error "RS_LD_LQ_ONLY requires M1_RS_IDENT_ENABLE or M1_RS_IDENT_USE_FIXED_NOMINAL"
#endif
#elif M1_LD_LQ_IDENT_ENABLE && M1_DEADBAND_FLOW_LD_LQ_AFTER_OPEN_UD
            if (s_id_cal_ld_lq_chain_pending != 0u) {
                s_id_cal_ld_lq_chain_pending = 0u;
                deadband_id_cal_begin_ld_lq_with_fixed_rs(ctx);
            } else {
                s_id_cal_state = M1_ID_CAL_ID_STEP;
                s_id_cal_tick = 0u;
                s_id_cal_id_step = 0u;
                ctx->id_ref = deadband_id_cal_ref_from_step(s_id_cal_id_step);
                deadband_id_cal_sweep_arm_bumpless(ctx->id_ref);
            }
#else
            s_id_cal_state = M1_ID_CAL_ID_STEP;
            s_id_cal_tick = 0u;
            s_id_cal_id_step = 0u;
            ctx->id_ref = deadband_id_cal_ref_from_step(s_id_cal_id_step);
            deadband_id_cal_sweep_arm_bumpless(ctx->id_ref);
#endif
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
#elif M1_RS_IDENT_ENABLE && M1_ID_CAL_COMMIT_LUT
            deadband_id_cal_pass0_commit_enter_rs_ident(ctx);
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
#if M1_ID_CAL_IQ_PROBE_LUT_AFTER_OFF
        if ((M1_ID_CAL_IQ_PROBE_OFF_S > 0.0f) &&
            (s_id_cal_tick >=
             deadband_id_cal_ticks_from_s(M1_ID_CAL_IQ_PROBE_OFF_S))) {
            if (M1_ID_CAL_IQ_PROBE_FIXED_S > 0.0f) {
                deadband_id_cal_enter_iq_probe_fixed(ctx);
            } else {
                deadband_id_cal_enter_iq_probe_on(ctx);
            }
        }
#endif
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

#if M1_RS_IDENT_ENABLE
    case M1_ID_CAL_RS_IDENT:
        ctx->iq_ref = 0.0f;
        ctx->id_ref = deadband_id_cal_clamp_id_ref(
            rs_ident_tick(ctx->id, ctx->ud_pi));
        if (rs_ident_is_done()) {
#if M1_LD_LQ_IDENT_ENABLE
            deadband_id_cal_begin_ld_lq_after_rs(ctx);
#else
            deadband_id_cal_enter_done(ctx);
#endif
        }
        break;
#endif

#if M1_LD_LQ_IDENT_ENABLE
    case M1_ID_CAL_LD_LQ_PRE_DECAY: {
        const uint32_t decay_ticks = deadband_id_cal_ticks_from_s(M1_LD_LQ_PRE_DECAY_S);

        ctx->id_ref = 0.0f;
        ctx->iq_ref = 0.0f;
        s_id_cal_tick++;
        if (decay_ticks == 0u || s_id_cal_tick >= decay_ticks) {
            deadband_id_cal_leave_ld_lq_pre_decay(ctx);
        }
        break;
    }

#if M1_LD_LQ_MULTI_ANGLE_ENABLE
    case M1_ID_CAL_LD_LQ_ALIGN: {
        const uint32_t align_ticks = deadband_id_cal_ticks_from_s(M1_LD_LQ_ALIGN_S);

        ctx->id_ref = 0.0f;
        ctx->iq_ref = 0.0f;
        s_id_cal_tick++;
        if (align_ticks == 0u || s_id_cal_tick >= align_ticks) {
            deadband_id_cal_start_ld_lq_ident(ctx, s_ld_lq_rs_cached);
        }
        break;
    }
#endif

    case M1_ID_CAL_LD_LQ_IDENT: {
        float id_ref = 0.0f;
        float iq_ref = 0.0f;

        ld_lq_ident_tick(ctx->id, ctx->iq, ctx->ud_pi, ctx->uq_pi,
                         &id_ref, &iq_ref);
        ctx->id_ref = deadband_id_cal_clamp_id_ref(id_ref);
        ctx->iq_ref = deadband_id_cal_clamp_iq_ref(iq_ref);
        if (ld_lq_ident_is_done()) {
            deadband_id_cal_ld_lq_leg_done(ctx);
        }
        break;
    }
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
    return deadband_id_cal_is_running() && !deadband_id_cal_in_iq_probe();
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
/* use_fix_theta / target_theta / align_ud*: stubs in deadband_id_cal.h */
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
/* capture_append_dlut: stub in deadband_id_cal.h when !DUAL_ANGLE */
void deadband_id_cal_clear_capture_pending(void) {}
void deadband_id_cal_sync_dbg(void) {}

#endif /* M1_ID_LOCK_CAL_SWEEP */
