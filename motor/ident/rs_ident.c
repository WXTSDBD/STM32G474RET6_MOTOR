/**
 * @file rs_ident.c
 * @date 2026-10-06
 * @brief 电阻辨识实现。

 *
 * 节拍限制见 rs_ident.h 文件头。
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
 */

#include "rs_ident.h"

#if M1_RS_IDENT_ENABLE

#include <math.h>

#include "dbg_monitor.h"
#include "motor_params_m1.h"

typedef enum {
    RS_SUB_RAMP_UP = 0,
    RS_SUB_RAMP_DOWN,
    RS_SUB_HOLD_ZERO,
    RS_SUB_DONE,
} rs_sub_state_t;

static rs_sub_state_t s_sub;
static float s_id_ref;
static uint8_t s_round;
static uint8_t s_done;
static uint8_t s_ok;
static uint32_t s_hold_tick;

static float s_sum_id;
static float s_sum_ud;
static float s_sum_id2;
static float s_sum_idud;
static uint32_t s_m;

static rs_ident_result_t s_result;

static uint32_t rs_ident_ticks_from_s(float s)
{
    if (s <= 0.0f) {
        return 0u;
    }
    return (uint32_t)(s / M1_CTRL_TS_S + 0.5f);
}

/** abc duty：LUT 已在占空比；稳态 Ud_pi 斜率 ≈ Rs。 */
static float rs_ident_u_ohm(float ud_pi, float id_fb)
{
    (void)id_fb;
    return ud_pi;
}

static void rs_ident_try_accumulate(float id_fb, float ud_pi)
{
    const float id_abs = (id_fb >= 0.0f) ? id_fb : -id_fb;
    const float u = rs_ident_u_ohm(ud_pi, id_fb);

    if (id_abs < M1_RS_IDENT_I_MIN_FIT_A) {
        return;
    }
    if (fabsf(id_fb - s_id_ref) > M1_RS_IDENT_EPS_TRACK_A) {
        return;
    }

    s_sum_id += id_fb;
    s_sum_ud += u;
    s_sum_id2 += id_fb * id_fb;
    s_sum_idud += id_fb * u;
    s_m++;
}

static void rs_ident_compute(void)
{
    float den;

    s_result.n = s_m;
    s_result.rs_nominal = M1_RS_OHM;
    s_result.repeat_n = (uint8_t)M1_RS_IDENT_REPEAT_N;
    s_result.rs_ohm = 0.0f;
    s_result.intercept_v = 0.0f;
    s_result.ok = 0u;
    s_ok = 0u;

    if (s_m < M1_RS_IDENT_MIN_SAMPLES) {
        return;
    }

    den = (float)s_m * s_sum_id2 - s_sum_id * s_sum_id;
    if (fabsf(den) < 1e-6f) {
        return;
    }

    s_result.rs_ohm =
        ((float)s_m * s_sum_idud - s_sum_id * s_sum_ud) / den;
    s_result.intercept_v =
        (s_sum_ud - s_result.rs_ohm * s_sum_id) / (float)s_m;
    s_result.ok = 1u;
    s_ok = 1u;
}

void rs_ident_init(void)
{
    s_sub = RS_SUB_DONE;
    s_id_ref = 0.0f;
    s_round = 0u;
    s_done = 1u;
    s_ok = 0u;
    s_hold_tick = 0u;
    s_sum_id = 0.0f;
    s_sum_ud = 0.0f;
    s_sum_id2 = 0.0f;
    s_sum_idud = 0.0f;
    s_m = 0u;
    s_result = (rs_ident_result_t){0};
}

void rs_ident_arm(void)
{
    s_sub = RS_SUB_RAMP_UP;
    s_id_ref = 0.0f;
    s_round = 0u;
    s_done = 0u;
    s_ok = 0u;
    s_hold_tick = 0u;
    s_sum_id = 0.0f;
    s_sum_ud = 0.0f;
    s_sum_id2 = 0.0f;
    s_sum_idud = 0.0f;
    s_m = 0u;
    s_result = (rs_ident_result_t){0};
}

float rs_ident_tick(float id_fb, float ud_pi)
{
    const float dt = M1_CTRL_TS_S;
    const float rate = M1_RS_IDENT_RAMP_A_PER_S;
    const uint32_t hold_ticks = rs_ident_ticks_from_s(M1_RS_IDENT_INTER_ROUND_S);

    if (s_done) {
        return 0.0f;
    }

    switch (s_sub) {
    case RS_SUB_RAMP_UP:
        s_id_ref += rate * dt;
        if (s_id_ref >= M1_RS_IDENT_I_MAX_A) {
            s_id_ref = M1_RS_IDENT_I_MAX_A;
            s_sub = RS_SUB_RAMP_DOWN;
        }
        rs_ident_try_accumulate(id_fb, ud_pi);
        break;

    case RS_SUB_RAMP_DOWN:
        s_id_ref -= rate * dt;
        if (s_id_ref <= 0.0f) {
            s_id_ref = 0.0f;
            s_sub = RS_SUB_HOLD_ZERO;
            s_hold_tick = 0u;
        }
        rs_ident_try_accumulate(id_fb, ud_pi);
        break;

    case RS_SUB_HOLD_ZERO:
        s_id_ref = 0.0f;
        s_hold_tick++;
        if (hold_ticks == 0u || s_hold_tick >= hold_ticks) {
            s_round++;
            if (s_round >= M1_RS_IDENT_REPEAT_N) {
                rs_ident_compute();
                rs_ident_sync_dbg();
                s_sub = RS_SUB_DONE;
                s_done = 1u;
            } else {
                s_sub = RS_SUB_RAMP_UP;
            }
        }
        break;

    case RS_SUB_DONE:
    default:
        s_done = 1u;
        s_id_ref = 0.0f;
        break;
    }

    return s_id_ref;
}

uint8_t rs_ident_is_done(void)
{
    return s_done;
}

uint8_t rs_ident_is_ok(void)
{
    return s_ok;
}

uint8_t rs_ident_open_seq_phase(void)
{
    if (!s_done) {
        return 54u;
    }
    return s_ok ? 55u : 56u;
}

void rs_ident_get_result(rs_ident_result_t *out)
{
    if (out != 0) {
        *out = s_result;
    }
}

void rs_ident_sync_dbg(void)
{
    dbg.rs_ident_ohm = s_result.rs_ohm;
    dbg.rs_ident_intercept_v = s_result.intercept_v;
    dbg.rs_ident_n = (float)s_result.n;
    dbg.rs_ident_ok = s_result.ok;
    dbg.rs_ident_round = s_round;
}

#endif /* M1_RS_IDENT_ENABLE */
