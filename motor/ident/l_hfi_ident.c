/**
 * @file l_hfi_ident.c
 * @date 2026-10-26
 * @brief 对齐 VESC mcpwm_foc_measure_inductance：旋转 HF + inv_L DFT。
 *
 * 流程：settle → duty 爬升（每档扫满 N 角）→ 多圈 meas →
 * bin0/bin2 → Ld=1/(off+amp)、Lq=1/(off-amp)、×0.9；
 * 可 REPEAT_N 轮（轮间 INTER 静置），ok 轮取均值。
 * 每轮收口后 SHOUT：mark55 连发本轮 Ld/Lq/Lavg（µH），再 INTER 或收尾。
 * 注入在固定 Park 下旋转 (ud,uq)，等效定子电压转角；基波电流为 0。
 */
#include "l_hfi_ident.h"

#include <math.h>
#include <string.h>

#include "dbg_monitor.h"
#include "motor_cfg.h"
#include "motor_params_m1.h"

#ifndef M1_L_HFI_DUTY_START
#define M1_L_HFI_DUTY_START         0.02f
#endif
#ifndef M1_L_HFI_DUTY_GROW
#define M1_L_HFI_DUTY_GROW          1.5f
#endif
#ifndef M1_L_HFI_DUTY_MAX
#define M1_L_HFI_DUTY_MAX           0.35f
#endif
#ifndef M1_L_HFI_I_GOAL_A
#define M1_L_HFI_I_GOAL_A           1.5f
#endif
#ifndef M1_L_HFI_MIN_DI_A
#define M1_L_HFI_MIN_DI_A           0.01f
#endif
#ifndef M1_L_HFI_N_ANG
#define M1_L_HFI_N_ANG              32u
#endif
#ifndef M1_L_HFI_MEAS_LAPS
#define M1_L_HFI_MEAS_LAPS          20u
#endif
#ifndef M1_L_HFI_SETTLE_S
#define M1_L_HFI_SETTLE_S           0.05f
#endif
#ifndef M1_L_HFI_GAP_S
#define M1_L_HFI_GAP_S              0.01f
#endif
#ifndef M1_L_HFI_SCALE
#define M1_L_HFI_SCALE              0.9f
#endif
#ifndef M1_L_HFI_L_MIN_H
#define M1_L_HFI_L_MIN_H            20e-6f
#endif
#ifndef M1_L_HFI_L_MAX_H
#define M1_L_HFI_L_MAX_H            0.001f
#endif
#ifndef M1_L_HFI_MIN_VALID_BINS
#define M1_L_HFI_MIN_VALID_BINS     28u
#endif
#ifndef M1_L_HFI_REPEAT_N
#define M1_L_HFI_REPEAT_N           1u
#endif
#ifndef M1_L_HFI_INTER_ROUND_S
#define M1_L_HFI_INTER_ROUND_S      0.0f
#endif
#ifndef M1_L_HFI_RESULT_SHOUT_N
/** 每轮结果 mark55 连发帧数（20 kHz 下 80≈4 ms，CSV 必能抓到）。 */
#define M1_L_HFI_RESULT_SHOUT_N     80u
#endif

#define L_HFI_N                     ((int)M1_L_HFI_N_ANG)

typedef enum {
    L_HFI_SUB_SETTLE = 0,
    L_HFI_SUB_SEEK,
    L_HFI_SUB_GAP,
    L_HFI_SUB_MEAS,
    L_HFI_SUB_SHOUT, /* 本轮 Ld/Lq/Lavg 连发，桥压 0 */
    L_HFI_SUB_INTER, /* 轮间静置：桥压 0，等若干秒再 seek */
    L_HFI_SUB_DONE
} l_hfi_sub_t;

/** shout 结束后：0=进 INTER，1=收尾均值并 DONE */
typedef enum {
    L_HFI_AFTER_SHOUT_INTER = 0,
    L_HFI_AFTER_SHOUT_FINAL
} l_hfi_after_shout_t;

static l_hfi_sub_t s_sub;
static uint8_t s_done;
static uint8_t s_ok;
static float s_rs_ohm;
static float s_duty;
static float s_v_inj;
static float s_u_d;
static float s_u_q;
/** 1=本拍输出 +Vh，采样后切到 −；0=本拍 −Vh，采样后记 prev。 */
static uint8_t s_pos_half;
static uint8_t s_ang;
static float s_i_prev;
static float s_di_abs_max;
static float s_inv_buf[L_HFI_N];
static float s_di_buf[L_HFI_N];
static uint8_t s_inv_hit[L_HFI_N];
/** 1=刚 reset_lap：本拍只保持 −V，等下一拍再记 i_prev（避免 settle/gap 0V 污染角0）。 */
static uint8_t s_prime;
static uint8_t s_lap;
static uint8_t s_laps_need;
static float s_l_sum;
static float s_diff_sum;
static float s_i_sum;
static float s_iter;
static float s_ld_h;
static float s_lq_h;
static float s_l_avg_h;
static float s_diff_h;
static uint32_t s_settle_left;
static l_hfi_ident_result_t s_result;
/** 当前轮序号 0..REPEAT_N-1 */
static uint8_t s_round;
static uint8_t s_rounds_need;
/** 各轮累加（仅 ok 轮计入），收尾取均值 */
static float s_acc_ld;
static float s_acc_lq;
static float s_acc_lavg;
static float s_acc_diff;
static float s_acc_v;
static uint8_t s_acc_n;
/** 上一圈有效 bin 数（写 inv 的角数），供遥测/判据对照 */
static uint8_t s_last_n_di;
/** 本轮结果连发：Ld/Lq/Lavg 单位 µH */
static float s_shout_ld_uH;
static float s_shout_lq_uH;
static float s_shout_lavg_uH;
static float s_shout_round;
static float s_shout_ok;
static uint16_t s_shout_left;
static l_hfi_after_shout_t s_after_shout;

/** cos(2π k / N)、sin；N=32 时 2θ 用 cos(4π k/N)。 */
static const float s_c1[L_HFI_N] = {
    1.000000000f, 0.980785280f, 0.923879533f, 0.831469612f,
    0.707106781f, 0.555570233f, 0.382683432f, 0.195090322f,
    0.000000000f, -0.195090322f, -0.382683432f, -0.555570233f,
    -0.707106781f, -0.831469612f, -0.923879533f, -0.980785280f,
    -1.000000000f, -0.980785280f, -0.923879533f, -0.831469612f,
    -0.707106781f, -0.555570233f, -0.382683432f, -0.195090322f,
    0.000000000f, 0.195090322f, 0.382683432f, 0.555570233f,
    0.707106781f, 0.831469612f, 0.923879533f, 0.980785280f
};
static const float s_s1[L_HFI_N] = {
    0.000000000f, 0.195090322f, 0.382683432f, 0.555570233f,
    0.707106781f, 0.831469612f, 0.923879533f, 0.980785280f,
    1.000000000f, 0.980785280f, 0.923879533f, 0.831469612f,
    0.707106781f, 0.555570233f, 0.382683432f, 0.195090322f,
    0.000000000f, -0.195090322f, -0.382683432f, -0.555570233f,
    -0.707106781f, -0.831469612f, -0.923879533f, -0.980785280f,
    -1.000000000f, -0.980785280f, -0.923879533f, -0.831469612f,
    -0.707106781f, -0.555570233f, -0.382683432f, -0.195090322f
};
/* cos(4π k/N)=cos(2π k/(N/2))：对 N=32 即 c1 的偶数索引步进 → 用 c1[(2k)%N] */
static const float s_c2[L_HFI_N] = {
    1.000000000f, 0.923879533f, 0.707106781f, 0.382683432f,
    0.000000000f, -0.382683432f, -0.707106781f, -0.923879533f,
    -1.000000000f, -0.923879533f, -0.707106781f, -0.382683432f,
    0.000000000f, 0.382683432f, 0.707106781f, 0.923879533f,
    1.000000000f, 0.923879533f, 0.707106781f, 0.382683432f,
    0.000000000f, -0.382683432f, -0.707106781f, -0.923879533f,
    -1.000000000f, -0.923879533f, -0.707106781f, -0.382683432f,
    0.000000000f, 0.382683432f, 0.707106781f, 0.923879533f
};
static const float s_s2[L_HFI_N] = {
    0.000000000f, 0.382683432f, 0.707106781f, 0.923879533f,
    1.000000000f, 0.923879533f, 0.707106781f, 0.382683432f,
    0.000000000f, -0.382683432f, -0.707106781f, -0.923879533f,
    -1.000000000f, -0.923879533f, -0.707106781f, -0.382683432f,
    0.000000000f, 0.382683432f, 0.707106781f, 0.923879533f,
    1.000000000f, 0.923879533f, 0.707106781f, 0.382683432f,
    0.000000000f, -0.382683432f, -0.707106781f, -0.923879533f,
    -1.000000000f, -0.923879533f, -0.707106781f, -0.382683432f
};

static uint32_t l_hfi_ticks_from_s(float s)
{
    if (s <= 0.0f) {
        return 0u;
    }
    return (uint32_t)(s / M1_CTRL_TS_S + 0.5f);
}

static float l_hfi_v_from_duty(float duty)
{
    return duty * M1_VBUS_V * (2.0f / 3.0f) * 0.8660254037844386f;
}

static float l_hfi_f_zv(void)
{
    return 1.0f / M1_CTRL_TS_S;
}

static void l_hfi_clear_outputs(void)
{
    s_u_d = 0.0f;
    s_u_q = 0.0f;
}

static void l_hfi_apply_volt(void)
{
    const float c = s_c1[s_ang];
    const float sn = s_s1[s_ang];
    const float v = (s_pos_half != 0u) ? s_v_inj : -s_v_inj;

    /* 固定 Park 下旋转电压矢量 = 定子侧转角扫掠（VESC 六矢量同形） */
    s_u_d = v * c;
    s_u_q = v * sn;
}

static float l_hfi_i_along(float id_fb, float iq_fb)
{
    return id_fb * s_c1[s_ang] + iq_fb * s_s1[s_ang];
}

static uint8_t l_hfi_l_valid(float l_h)
{
    return (l_h >= M1_L_HFI_L_MIN_H && l_h < M1_L_HFI_L_MAX_H) ? 1u : 0u;
}

/**
 * @brief 对 inv_L 缓冲做 bin0 / bin2（长度归一，bin2 幅度×2 对齐 VESC）。
 */
static void l_hfi_dft_ld_lq(const float *buf, float *ld_h, float *lq_h,
                            float *l_avg_h, float *diff_h)
{
    float off = 0.0f;
    float re2 = 0.0f;
    float im2 = 0.0f;
    float amp;
    int k;

    for (k = 0; k < L_HFI_N; k++) {
        off += buf[k];
        re2 += buf[k] * s_c2[k];
        im2 += buf[k] * s_s2[k];
    }
    off /= (float)L_HFI_N;
    re2 /= (float)L_HFI_N;
    im2 /= (float)L_HFI_N;
    /* dual-sided 归一化：幅度取 2·|X2| */
    amp = 2.0f * sqrtf(re2 * re2 + im2 * im2);

    if (off <= amp + 1e-3f) {
        *ld_h = 0.0f;
        *lq_h = 0.0f;
        *l_avg_h = 0.0f;
        *diff_h = 0.0f;
        return;
    }
    *ld_h = 1.0f / (off + amp);
    *lq_h = 1.0f / (off - amp);
    *l_avg_h = 0.5f * (*ld_h + *lq_h);
    *diff_h = *lq_h - *ld_h;
}

/**
 * @brief 武装本轮结果连发（mark55）；结束后进 INTER 或收尾。
 */
static void l_hfi_arm_round_shout(uint8_t round_1based, uint8_t round_ok,
                                 l_hfi_after_shout_t after)
{
    s_shout_ld_uH = s_ld_h * 1e6f;
    s_shout_lq_uH = s_lq_h * 1e6f;
    s_shout_lavg_uH = s_l_avg_h * 1e6f;
    s_shout_round = (float)round_1based;
    s_shout_ok = (round_ok != 0u) ? 1.0f : 0.0f;
    s_shout_left = (uint16_t)M1_L_HFI_RESULT_SHOUT_N;
    if (s_shout_left < 1u) {
        s_shout_left = 1u;
    }
    s_after_shout = after;
    s_sub = L_HFI_SUB_SHOUT;
    l_hfi_clear_outputs();
    l_hfi_ident_sync_dbg();
}

/**
 * @brief 三轮累加收尾：写最终均值，DONE。
 */
static void l_hfi_finalize_all(void)
{
    float inv_n;

    if (s_acc_n > 0u) {
        inv_n = 1.0f / (float)s_acc_n;
        s_ld_h = s_acc_ld * inv_n;
        s_lq_h = s_acc_lq * inv_n;
        s_l_avg_h = s_acc_lavg * inv_n;
        s_diff_h = s_acc_diff * inv_n;
        s_v_inj = s_acc_v * inv_n;
        s_result.l_avg_h = s_l_avg_h;
        s_result.ld_lq_diff_h = s_diff_h;
        s_result.ld_h = s_ld_h;
        s_result.lq_h = s_lq_h;
        s_result.v_inj_used = s_v_inj;
        s_result.ok = 1u;
        s_ok = 1u;
    } else {
        s_result.ok = 0u;
        s_ok = 0u;
    }
    s_result.rs_used_ohm = s_rs_ohm;
    s_done = 1u;
    s_sub = L_HFI_SUB_DONE;
    s_shout_left = 0u;
    l_hfi_clear_outputs();
    l_hfi_ident_sync_dbg();
}

/**
 * @brief 本轮 meas 收口：有效则计入累加；先 shout 本轮，再 INTER 或收尾。
 */
static void l_hfi_finish_round(void)
{
    float inv_n;
    uint8_t round_ok = 0u;

    if (s_iter >= 0.5f) {
        inv_n = 1.0f / s_iter;
        s_l_avg_h = (s_l_sum * inv_n) * M1_L_HFI_SCALE;
        s_diff_h = (s_diff_sum * inv_n) * M1_L_HFI_SCALE;
        s_ld_h = s_l_avg_h - 0.5f * s_diff_h;
        s_lq_h = s_l_avg_h + 0.5f * s_diff_h;
        round_ok = (l_hfi_l_valid(s_ld_h) && l_hfi_l_valid(s_lq_h)) ? 1u : 0u;
        if (round_ok != 0u) {
            s_acc_ld += s_ld_h;
            s_acc_lq += s_lq_h;
            s_acc_lavg += s_l_avg_h;
            s_acc_diff += s_diff_h;
            s_acc_v += s_v_inj;
            s_acc_n++;
        }
    } else {
        s_ld_h = 0.0f;
        s_lq_h = 0.0f;
        s_l_avg_h = 0.0f;
        s_diff_h = 0.0f;
    }

    s_result.l_avg_h = s_l_avg_h;
    s_result.ld_lq_diff_h = s_diff_h;
    s_result.ld_h = s_ld_h;
    s_result.lq_h = s_lq_h;
    s_result.rs_used_ohm = s_rs_ohm;
    s_result.v_inj_used = s_v_inj;
    s_result.ok = round_ok;
    s_ok = round_ok;

    s_round++;
    if (s_round < s_rounds_need) {
        l_hfi_arm_round_shout(s_round, round_ok, L_HFI_AFTER_SHOUT_INTER);
    } else {
        l_hfi_arm_round_shout(s_round, round_ok, L_HFI_AFTER_SHOUT_FINAL);
    }
}

/**
 * @brief 从当前 duty 配置重新 seek（新一轮，不重置累加）。
 */
static void l_hfi_start_seek_round(void)
{
    s_lap = 0u;
    s_l_sum = 0.0f;
    s_diff_sum = 0.0f;
    s_i_sum = 0.0f;
    s_iter = 0.0f;
    s_duty = M1_L_HFI_DUTY_START;
    s_v_inj = l_hfi_v_from_duty(s_duty);
    memset(s_inv_buf, 0, sizeof(s_inv_buf));
    memset(s_inv_hit, 0, sizeof(s_inv_hit));
    memset(s_di_buf, 0, sizeof(s_di_buf));
    s_settle_left = l_hfi_ticks_from_s(M1_L_HFI_SETTLE_S);
    s_sub = L_HFI_SUB_SETTLE;
    l_hfi_clear_outputs();
}

static void l_hfi_reset_lap(void)
{
    s_ang = 0u;
    s_pos_half = 0u;
    s_i_prev = 0.0f;
    s_di_abs_max = 0.0f;
    /* inv_buf 保留上圈（VESC 环形覆盖）；本圈 di/hit 清零 */
    memset(s_di_buf, 0, sizeof(s_di_buf));
    memset(s_inv_hit, 0, sizeof(s_inv_hit));
    s_prime = 1u;
    l_hfi_apply_volt();
}

static void l_hfi_on_lap_done(void)
{
    float ld;
    float lq;
    float lavg;
    float diff;
    float di_mean = 0.0f;
    int k;
    int n_di = 0;

    for (k = 0; k < L_HFI_N; k++) {
        if (s_inv_hit[k] != 0u) {
            di_mean += s_di_buf[k];
            n_di++;
        }
    }
    s_last_n_di = (uint8_t)n_di;
    if (n_di > 0) {
        di_mean /= (float)n_di;
    }
    if (di_mean > s_di_abs_max) {
        s_di_abs_max = di_mean;
    }

    if (s_sub == L_HFI_SUB_SEEK) {
        if (s_di_abs_max >= M1_L_HFI_I_GOAL_A) {
            s_settle_left = l_hfi_ticks_from_s(M1_L_HFI_GAP_S);
            s_sub = L_HFI_SUB_GAP;
            l_hfi_clear_outputs();
            return;
        }
        if (s_duty < M1_L_HFI_DUTY_MAX * 0.999f) {
            s_duty *= M1_L_HFI_DUTY_GROW;
            if (s_duty > M1_L_HFI_DUTY_MAX) {
                s_duty = M1_L_HFI_DUTY_MAX;
            }
            s_v_inj = l_hfi_v_from_duty(s_duty);
            s_di_abs_max = 0.0f;
            l_hfi_reset_lap();
            return;
        }
        /* duty 顶仍未达标：仍进 meas */
        s_settle_left = l_hfi_ticks_from_s(M1_L_HFI_GAP_S);
        s_sub = L_HFI_SUB_GAP;
        l_hfi_clear_outputs();
        return;
    }

    /* MEAS：有效 bin 不足则丢本圈（避免 0 填充拉低 inv_L） */
    if (n_di >= (int)M1_L_HFI_MIN_VALID_BINS) {
        l_hfi_dft_ld_lq(s_inv_buf, &ld, &lq, &lavg, &diff);
        if (l_hfi_l_valid(ld) && l_hfi_l_valid(lq)) {
            s_l_sum += lavg;
            s_diff_sum += diff;
            s_i_sum += di_mean;
            s_iter += 1.0f;
        }
    }
    s_lap++;
    if (s_lap >= s_laps_need) {
        l_hfi_finish_round();
        return;
    }
    l_hfi_reset_lap();
}

/**
 * @brief 一拍：先按上拍电压采样，再写出下一拍 ud/uq。
 */
static void l_hfi_step_sample(float id_fb, float iq_fb)
{
    const float i_now = l_hfi_i_along(id_fb, iq_fb);
    float di;
    float f_zv;

    if (s_prime != 0u) {
        /* cold-start：−V 本拍才真正打到桥，不记 i_prev、不翻号 */
        s_prime = 0u;
        s_pos_half = 0u;
        l_hfi_apply_volt();
        return;
    }

    if (s_pos_half == 0u) {
        /* −Vh 半拍：记起点，下一拍改 +Vh */
        s_i_prev = i_now;
        s_pos_half = 1u;
        l_hfi_apply_volt();
        return;
    }

    /* +Vh 半拍：Δi，写入本角，推进角。
     * 用 |di| 写 inv：SEEK 本就看 |di|；若要求 di>0，半拍顺序/投影符号
     * 稍偏就会 n_di<<32 → 达不到 MIN_VALID_BINS → 整轮 FAIL。 */
    di = i_now - s_i_prev;
    {
        const float adi = fabsf(di);

        if (adi > s_di_abs_max) {
            s_di_abs_max = adi;
        }
        s_di_buf[s_ang] = adi;
        if (adi > M1_L_HFI_MIN_DI_A && s_v_inj > 1e-4f) {
            f_zv = l_hfi_f_zv();
            s_inv_buf[s_ang] = (f_zv * adi) / s_v_inj;
            s_inv_hit[s_ang] = 1u;
        }
    }

    s_ang++;
    s_pos_half = 0u;
    if (s_ang >= (uint8_t)L_HFI_N) {
        l_hfi_on_lap_done();
        return;
    }
    l_hfi_apply_volt();
}

void l_hfi_ident_sync_dbg(void)
{
    dbg.ld_lq_ok = s_ok;
    dbg.ld_lq_v_inj = s_v_inj;
    dbg.ld_lq_rs_used = s_rs_ohm;
    dbg.ld_lq_axis = s_ang;
    dbg.ld_lq_sub = (uint8_t)s_sub;
    dbg.ld_lq_f_hz = l_hfi_f_zv();
    dbg.ld_lq_proc_di_den_a = s_di_abs_max;
    /* 复用：半拍计数槽改报上一圈有效 bin；round 在 amp_idx */
    dbg.ld_lq_proc_half_ticks = (float)s_last_n_di;
    dbg.ld_lq_proc_amp_idx = (float)s_round;
    /* shout 段强制贴本轮 µH（含 fail 轮，供 mark55 抓包） */
    if (s_sub == L_HFI_SUB_SHOUT) {
        dbg.ld_lq_proc_last_l_uH = s_shout_ld_uH;
        dbg.ld_lq_proc_psi_du_wb = s_shout_lq_uH;
        dbg.ld_lq_L_est_uH = s_shout_lavg_uH;
        dbg.ld_lq_proc_amp_idx = s_shout_round;
        dbg.ld_lq_proc_coarse_l_uH = s_shout_ok;
    } else if (s_ok != 0u) {
        dbg.ld_lq_proc_last_l_uH = s_ld_h * 1e6f;
        dbg.ld_lq_proc_psi_du_wb = s_lq_h * 1e6f;
        dbg.ld_lq_L_est_uH = s_l_avg_h * 1e6f;
        dbg.ld_lq_proc_coarse_l_uH = s_diff_h * 1e6f;
    }
}

void l_hfi_ident_init(void)
{
    s_sub = L_HFI_SUB_DONE;
    s_done = 1u;
    s_ok = 0u;
    s_rs_ohm = g_m1_motor_cfg.rs_ohm;
    s_duty = M1_L_HFI_DUTY_START;
    s_v_inj = 0.0f;
    l_hfi_clear_outputs();
    s_ld_h = 0.0f;
    s_lq_h = 0.0f;
    s_l_avg_h = 0.0f;
    s_diff_h = 0.0f;
    s_round = 0u;
    s_rounds_need = 1u;
    s_acc_n = 0u;
    s_last_n_di = 0u;
    s_shout_ld_uH = 0.0f;
    s_shout_lq_uH = 0.0f;
    s_shout_lavg_uH = 0.0f;
    s_shout_round = 0.0f;
    s_shout_ok = 0.0f;
    s_shout_left = 0u;
    s_after_shout = L_HFI_AFTER_SHOUT_FINAL;
    memset(&s_result, 0, sizeof(s_result));
    l_hfi_ident_sync_dbg();
}

void l_hfi_ident_arm(float rs_ohm)
{
    s_rs_ohm = (rs_ohm > 0.0f) ? rs_ohm : g_m1_motor_cfg.rs_ohm;
    s_done = 0u;
    s_ok = 0u;
    s_ld_h = 0.0f;
    s_lq_h = 0.0f;
    s_l_avg_h = 0.0f;
    s_diff_h = 0.0f;
    s_l_sum = 0.0f;
    s_diff_sum = 0.0f;
    s_i_sum = 0.0f;
    s_iter = 0.0f;
    s_lap = 0u;
    s_laps_need = (uint8_t)M1_L_HFI_MEAS_LAPS;
    s_rounds_need = (uint8_t)M1_L_HFI_REPEAT_N;
    if (s_rounds_need < 1u) {
        s_rounds_need = 1u;
    }
    s_round = 0u;
    s_acc_ld = 0.0f;
    s_acc_lq = 0.0f;
    s_acc_lavg = 0.0f;
    s_acc_diff = 0.0f;
    s_acc_v = 0.0f;
    s_acc_n = 0u;
    s_shout_ld_uH = 0.0f;
    s_shout_lq_uH = 0.0f;
    s_shout_lavg_uH = 0.0f;
    s_shout_round = 0.0f;
    s_shout_ok = 0.0f;
    s_shout_left = 0u;
    s_after_shout = L_HFI_AFTER_SHOUT_FINAL;
    s_duty = M1_L_HFI_DUTY_START;
    s_v_inj = l_hfi_v_from_duty(s_duty);
    s_prime = 0u;
    memset(s_inv_buf, 0, sizeof(s_inv_buf));
    memset(s_inv_hit, 0, sizeof(s_inv_hit));
    memset(s_di_buf, 0, sizeof(s_di_buf));
    memset(&s_result, 0, sizeof(s_result));
    s_result.rs_used_ohm = s_rs_ohm;
    s_settle_left = l_hfi_ticks_from_s(M1_L_HFI_SETTLE_S);
    s_sub = L_HFI_SUB_SETTLE;
    l_hfi_clear_outputs();
    l_hfi_ident_sync_dbg();
}

void l_hfi_ident_tick(float id_fb, float iq_fb)
{
    (void)id_fb;
    (void)iq_fb;

    if (s_done != 0u) {
        l_hfi_clear_outputs();
        return;
    }

    switch (s_sub) {
    case L_HFI_SUB_SETTLE:
        l_hfi_clear_outputs();
        if (s_settle_left > 0u) {
            s_settle_left--;
        }
        if (s_settle_left == 0u) {
            s_di_abs_max = 0.0f;
            l_hfi_reset_lap();
            s_sub = L_HFI_SUB_SEEK;
        }
        break;

    case L_HFI_SUB_GAP:
        l_hfi_clear_outputs();
        if (s_settle_left > 0u) {
            s_settle_left--;
        }
        if (s_settle_left == 0u) {
            s_lap = 0u;
            s_l_sum = 0.0f;
            s_diff_sum = 0.0f;
            s_i_sum = 0.0f;
            s_iter = 0.0f;
            l_hfi_reset_lap();
            s_sub = L_HFI_SUB_MEAS;
        }
        break;

    case L_HFI_SUB_SHOUT:
        l_hfi_clear_outputs();
        if (s_shout_left > 0u) {
            s_shout_left--;
        }
        if (s_shout_left == 0u) {
            if (s_after_shout == L_HFI_AFTER_SHOUT_INTER) {
                s_settle_left = l_hfi_ticks_from_s(M1_L_HFI_INTER_ROUND_S);
                s_sub = L_HFI_SUB_INTER;
            } else {
                l_hfi_finalize_all();
            }
        }
        break;

    case L_HFI_SUB_INTER:
        l_hfi_clear_outputs();
        if (s_settle_left > 0u) {
            s_settle_left--;
        }
        if (s_settle_left == 0u) {
            l_hfi_start_seek_round();
        }
        break;

    case L_HFI_SUB_SEEK:
    case L_HFI_SUB_MEAS:
        /* 电压由 on_sample 推进 */
        break;

    case L_HFI_SUB_DONE:
    default:
        l_hfi_clear_outputs();
        break;
    }
    l_hfi_ident_sync_dbg();
}

void l_hfi_ident_on_sample(float id_fb, float iq_fb)
{
    if ((s_sub != L_HFI_SUB_SEEK) && (s_sub != L_HFI_SUB_MEAS)) {
        return;
    }
    l_hfi_step_sample(id_fb, iq_fb);
}

float l_hfi_ident_u_d(void)
{
    return s_u_d;
}

float l_hfi_ident_u_q(void)
{
    return s_u_q;
}

uint8_t l_hfi_ident_inject_active(void)
{
    return ((s_sub == L_HFI_SUB_SEEK) || (s_sub == L_HFI_SUB_MEAS)) ? 1u : 0u;
}

uint8_t l_hfi_ident_is_done(void)
{
    return s_done;
}

uint8_t l_hfi_ident_is_ok(void)
{
    return s_ok;
}

uint8_t l_hfi_ident_open_seq_phase(void)
{
    if (s_done == 0u) {
        if (s_sub == L_HFI_SUB_SHOUT) {
            return 55u; /* 本轮结果包 */
        }
        if (s_sub == L_HFI_SUB_INTER) {
            return 56u;
        }
        return 57u;
    }
    return (s_ok != 0u) ? 58u : 59u;
}

void l_hfi_ident_get_result(l_hfi_ident_result_t *out)
{
    if (out != 0) {
        *out = s_result;
    }
}

uint8_t l_hfi_ident_result_shout_active(void)
{
    return ((s_done == 0u) && (s_sub == L_HFI_SUB_SHOUT)) ? 1u : 0u;
}

float l_hfi_ident_shout_ld_uH(void)
{
    return s_shout_ld_uH;
}

float l_hfi_ident_shout_lq_uH(void)
{
    return s_shout_lq_uH;
}

float l_hfi_ident_shout_lavg_uH(void)
{
    return s_shout_lavg_uH;
}

float l_hfi_ident_shout_round(void)
{
    return s_shout_round;
}

float l_hfi_ident_shout_ok(void)
{
    return s_shout_ok;
}
