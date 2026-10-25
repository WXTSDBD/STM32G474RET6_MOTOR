/**
 * @file exp_rev141.c
 * @date 2026-10-25
 * @brief ±2000 rpm 段表。无感 2 对，刹停，有感 1 对，再刹停。
 *
 * 目标 ±2000 rpm，到速后再保持 0.5 s。刹停 |n|≤80 rpm 再停 0.3 s，超时 3 s。
 * 斜坡不在段时长里，速度环读 route 的 500/800 rpm/s。
 * mark 不低于 0x30 才把角度源换成编码器。到速和刹停用 tick 带来的转速反馈。
 */
#include "experiment/exp_tables.h"

#include <stddef.h>

#include "core/motor_route.h"
#include "dbg_monitor.h"
#include "experiment/exp_act_basic.h"
#include "observer/observer_composite.h"

/* 段表字段顺序：{ dur_ms, act, ops, p0, p1, p2, cfg_idx, mark, guard_mask } */
#define SEG(dur, act, a, b, c, mark) \
    { (uint16_t)(dur), (uint8_t)(act), NULL, (a), (b), (c), 0u, (uint8_t)(mark), 0u }

static const exp_cfg_t s_cfg_default[] = {
    { 0u, 0u, 0u, 0u, 0.0f, 0.0f },
};

static const exp_seg_t s_rev141_segs[] = {
    SEG(0u,    EXP_ACT_OMEGA_REF, +2000.0f, 0.0f, 0.0f, 0x11),
    SEG(500u,  EXP_ACT_WAIT,       500.0f,  0.0f, 0.0f, 0x11),
    SEG(0u,    EXP_ACT_OMEGA_REF, -2000.0f, 0.0f, 0.0f, 0x12),
    SEG(500u,  EXP_ACT_WAIT,       500.0f,  0.0f, 0.0f, 0x12),
    SEG(0u,    EXP_ACT_OMEGA_REF, +2000.0f, 0.0f, 0.0f, 0x13),
    SEG(500u,  EXP_ACT_WAIT,       500.0f,  0.0f, 0.0f, 0x13),
    SEG(0u,    EXP_ACT_OMEGA_REF, -2000.0f, 0.0f, 0.0f, 0x14),
    SEG(500u,  EXP_ACT_WAIT,       500.0f,  0.0f, 0.0f, 0x14),
    SEG(3000u, EXP_ACT_STOP,        80.0f,  0.3f, 0.0f, 0x20),
    SEG(0u,    EXP_ACT_OMEGA_REF, +2000.0f, 0.0f, 0.0f, 0x31),
    SEG(500u,  EXP_ACT_WAIT,       500.0f,  0.0f, 0.0f, 0x31),
    SEG(0u,    EXP_ACT_OMEGA_REF, -2000.0f, 0.0f, 0.0f, 0x32),
    SEG(500u,  EXP_ACT_WAIT,       500.0f,  0.0f, 0.0f, 0x32),
    SEG(3000u, EXP_ACT_STOP,        80.0f,  0.3f, 0.0f, 0x33),
};

/** 外环 2 kHz，与 exp_runner 的节拍一致。 */
#define REV141_TICK_MS  0.5f

static uint16_t s_rev141_i;
static float s_rev141_elapsed_ms;
static float s_rev141_stop_hold_s;
static float s_rev141_cmd;
static uint8_t s_rev141_sensed_latched;
static uint8_t s_rev141_script_done;
static uint8_t s_rev141_started;

/**
 * @brief 进入当前段：写转速指令，并在首个有感段换角度源。
 * @param ctx 控制上下文。不可为 NULL。
 */
static void rev141_enter(motor_context_t *ctx)
{
    const exp_seg_t *seg = &s_rev141_segs[s_rev141_i];

    s_rev141_elapsed_ms = 0.0f;
    s_rev141_stop_hold_s = -1.0f;
    DBG.open_seq_phase = seg->mark;
    exp_act_angle_from_mark(seg->mark);
    if (seg->act == EXP_ACT_OMEGA_REF) {
        s_rev141_cmd = seg->p0;
    } else if (seg->act == EXP_ACT_STOP) {
        s_rev141_cmd = 0.0f;
    }
    /* 只写目标。斜坡状态留在速度环里，按 gains 的 rpm/s 爬。 */
    ctx->omega_ref = s_rev141_cmd;
    if ((seg->mark >= 0x30u) && (s_rev141_sensed_latched == 0u)) {
        motor_angle_use(&g_angle_enc);
        observer_rev141_enter_sensed();
        s_rev141_sensed_latched = 1u;
    }
}

/**
 * @brief 当前段结束条件已满足时推进。走完则转速指令归 0。
 * @param ctx 控制上下文。不可为 NULL。
 */
static void rev141_advance(motor_context_t *ctx)
{
    s_rev141_i++;
    if (s_rev141_i >= (uint16_t)(sizeof(s_rev141_segs) / sizeof(s_rev141_segs[0]))) {
        s_rev141_script_done = 1u;
        s_rev141_cmd = 0.0f;
        ctx->omega_ref = 0.0f;
        observer_rev141_note_done();
        return;
    }
    rev141_enter(ctx);
}

/**
 * @brief 接管转速指令并清段下标。角度源先回到无感。
 * @param ctx 未使用。
 * @param cfg 未使用。
 */
static void rev141_script_init(motor_context_t *ctx, const exp_cfg_t *cfg)
{
    (void)ctx;
    (void)cfg;
    s_rev141_i = 0u;
    s_rev141_sensed_latched = 0u;
    s_rev141_script_done = 0u;
    s_rev141_started = 0u;
    s_rev141_cmd = 0.0f;
    motor_angle_use(&g_angle_hfi);
}

/**
 * @brief 按段动作推进。OMEGA_REF 等到速，WAIT 等 p0 毫秒，STOP 等到停或超时。
 * @param ctx 控制上下文。
 * @param theta_fb_rad 未使用。
 * @param omega_rpm 速度环本拍反馈，单位 rpm。到速和刹停都用它。
 * @note 速度环未开时不发转速，避免起动阶段就把 ±2000 写进去。
 *       无感段这路是观测转速。换上编码器角度源之后才是编码器。
 */
static void rev141_script_tick(motor_context_t *ctx, float theta_fb_rad, float omega_rpm)
{
    const exp_seg_t *seg;
    float meas;

    (void)theta_fb_rad;
    if ((ctx == NULL) || (s_rev141_script_done != 0u)) {
        return;
    }
    if (s_rev141_started == 0u) {
        if (observer_speed_run_active() == 0u) {
            return;
        }
        s_rev141_started = 1u;
        rev141_enter(ctx);
    }
    if (s_rev141_i >= (uint16_t)(sizeof(s_rev141_segs) / sizeof(s_rev141_segs[0]))) {
        s_rev141_script_done = 1u;
        return;
    }
    seg = &s_rev141_segs[s_rev141_i];
    ctx->omega_ref = s_rev141_cmd;
    meas = omega_rpm;
    if (seg->act == EXP_ACT_OMEGA_REF) {
        if (((s_rev141_cmd > 0.0f) && (meas >= s_rev141_cmd)) ||
            ((s_rev141_cmd < 0.0f) && (meas <= s_rev141_cmd))) {
            rev141_advance(ctx);
        }
        return;
    }
    s_rev141_elapsed_ms += REV141_TICK_MS;
    if (seg->act == EXP_ACT_WAIT) {
        if (s_rev141_elapsed_ms >= seg->p0) {
            rev141_advance(ctx);
        }
        return;
    }
    if (seg->act == EXP_ACT_STOP) {
        if (meas < 0.0f) {
            meas = -meas;
        }
        if (meas <= seg->p0) {
            if (s_rev141_stop_hold_s < 0.0f) {
                s_rev141_stop_hold_s = 0.0f;
            } else {
                s_rev141_stop_hold_s += REV141_TICK_MS * 0.001f;
            }
        } else {
            s_rev141_stop_hold_s = -1.0f;
        }
        if ((s_rev141_stop_hold_s >= seg->p1) ||
            (s_rev141_elapsed_ms >= (float)seg->dur_ms)) {
            rev141_advance(ctx);
        }
    }
}

/**
 * @brief 1=段表走完。
 * @param ctx 未使用。
 */
static uint8_t rev141_script_done(const motor_context_t *ctx)
{
    (void)ctx;
    return s_rev141_script_done;
}

static const exp_script_ops_t s_rev141_script = {
    rev141_script_init,
    rev141_script_tick,
    rev141_script_done,
};

const exp_def_t g_exp_rev141 = {
    EXP_ID_REV141,
    "rev141_2000rpm",
    EXP_KIND_SCRIPT,
    s_rev141_segs,
    (uint16_t)(sizeof(s_rev141_segs) / sizeof(s_rev141_segs[0])),
    s_cfg_default,
    1u,
    0u,
    &s_rev141_script,
    1u,
};
