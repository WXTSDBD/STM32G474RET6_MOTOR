/**
 * @file deadband_cal.c
 * @date 2026-10-06
 * @brief 扫表采样、排序去重、几何建表和 NVM 导出。

 *
 * 节拍限制见 deadband_cal.h 文件头。
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
 */

#include "deadband_cal.h"

#include <math.h>
#include <stddef.h>

#include "deadband.h"
#include "deadband_geo.h"
#include "factory_nvm.h"
#include "motor_cfg.h"
#include "motor_params_m1.h"

#include <string.h>

/** NVM 运行时缓冲（NORMAL 上电加载；与 sweep RAM 分离） */
static float s_nvm_cluster_amps[2][M1_DEADBAND_LUT_MAX];
static float s_nvm_cluster_vals[2][M1_DEADBAND_LUT_MAX];
static uint8_t s_nvm_len;
static uint8_t s_nvm_two_cluster;
static uint8_t s_nvm_loaded;

static float deadband_cal_lut_interp_v(const float *amps, const float *vals,
                                         uint8_t len, float i_abs)
{
    uint8_t i;

    if (amps == NULL || vals == NULL || len < 2u) {
        return 0.0f;
    }

    if (i_abs <= amps[0]) {
        return vals[0];
    }
    if (i_abs >= amps[len - 1u]) {
        return vals[len - 1u];
    }

    for (i = 0u; i < (len - 1u); i++) {
        if (i_abs >= amps[i] && i_abs <= amps[i + 1u]) {
            float t = (i_abs - amps[i]) / (amps[i + 1u] - amps[i]);
            return vals[i] + t * (vals[i + 1u] - vals[i]);
        }
    }

    return vals[len - 1u];
}

static void deadband_cal_pack_nvm_record(factory_nvm_deadband_t *out,
                                         const float *a0, const float *v0,
                                         const float *a1, const float *v1,
                                         uint8_t len, uint8_t two_cluster)
{
    memset(out, 0, sizeof(*out));
    out->valid = 1u;
    out->len = len;
    out->two_cluster = two_cluster;
    out->runtime_scale = deadband_get_lut_runtime_scale();
    memcpy(out->amps_a, a0, (size_t)len * sizeof(float));
    memcpy(out->vals_a, v0, (size_t)len * sizeof(float));
    if (two_cluster != 0u) {
        memcpy(out->amps_b, a1, (size_t)len * sizeof(float));
        memcpy(out->vals_b, v1, (size_t)len * sizeof(float));
    }
}

#if M1_ID_LOCK_CAL_SWEEP
static bool deadband_cal_export_nvm_sweep(factory_nvm_deadband_t *out);
#endif

bool deadband_cal_export_nvm(factory_nvm_deadband_t *out)
{
    if (out == NULL) {
        return false;
    }

#if M1_ID_LOCK_CAL_SWEEP
    return deadband_cal_export_nvm_sweep(out);
#else
    const float *a0;
    const float *v0;
    const float *a1;
    const float *v1;
    uint8_t len;
    uint8_t two_cluster;

    if (s_nvm_loaded == 0u || s_nvm_len < 2u) {
        return false;
    }
    len = s_nvm_len;
    a0 = s_nvm_cluster_amps[0];
    v0 = s_nvm_cluster_vals[0];
    a1 = s_nvm_cluster_amps[1];
    v1 = s_nvm_cluster_vals[1];
    two_cluster = s_nvm_two_cluster;
    deadband_cal_pack_nvm_record(out, a0, v0, a1, v1, len, two_cluster);
    return true;
#endif
}

bool deadband_cal_apply_nvm(const factory_nvm_deadband_t *in)
{
    uint8_t len;

    if (in == NULL || in->valid == 0u || in->len < 2u) {
        return false;
    }

    len = in->len;
    if (len > M1_DEADBAND_LUT_MAX) {
        len = M1_DEADBAND_LUT_MAX;
    }

    memcpy(s_nvm_cluster_amps[0], in->amps_a, (size_t)len * sizeof(float));
    memcpy(s_nvm_cluster_vals[0], in->vals_a, (size_t)len * sizeof(float));
    if (in->two_cluster != 0u) {
        memcpy(s_nvm_cluster_amps[1], in->amps_b, (size_t)len * sizeof(float));
        memcpy(s_nvm_cluster_vals[1], in->vals_b, (size_t)len * sizeof(float));
    }
    s_nvm_len = len;
    s_nvm_two_cluster = in->two_cluster;
    s_nvm_loaded = 1u;

    deadband_set_lut(s_nvm_cluster_amps[0], s_nvm_cluster_vals[0], len);
    deadband_set_lut_domain(0u);
    deadband_set_runtime_apply_ud(0u);
#if M1_DEADBAND_GEO_TWO_CLUSTER_ENABLE
    if (in->two_cluster != 0u) {
        deadband_set_cluster_luts(s_nvm_cluster_amps[0], s_nvm_cluster_vals[0],
                                  s_nvm_cluster_amps[1], s_nvm_cluster_vals[1],
                                  len);
    }
#endif
    if (in->runtime_scale > 0.01f) {
        deadband_set_lut_runtime_scale(in->runtime_scale);
    } else {
        deadband_set_lut_runtime_scale(M1_DEADBAND_LUT_RUNTIME_SCALE);
    }
    deadband_set_mode(M1_DEADBAND_MODE_LUT);
    return true;
}

#if M1_ID_LOCK_CAL_SWEEP

#ifndef M1_ID_CAL_CAPTURE_EPS_A
#define M1_ID_CAL_CAPTURE_EPS_A    0.03f
#endif

#ifndef M1_ID_CAL_OUTLIER_V
#define M1_ID_CAL_OUTLIER_V        1.0f
#endif

/** 段 2=0：只 capture；段 3=1：commit 时 deadband_set_lut */
#ifndef M1_ID_CAL_COMMIT_LUT
#define M1_ID_CAL_COMMIT_LUT       0
#endif

static float s_dlut_amps[M1_DEADBAND_LUT_MAX];
static float s_dlut_vals[M1_DEADBAND_LUT_MAX];
static float s_plut_amps[3][M1_DEADBAND_LUT_MAX];
static float s_plut_vals[3][M1_DEADBAND_LUT_MAX];
static float s_plut_cluster_amps[2][M1_DEADBAND_LUT_MAX];
static float s_plut_cluster_vals[2][M1_DEADBAND_LUT_MAX];
static uint8_t s_plut_triplet;
static uint8_t s_plut_cluster_valid;
static uint8_t s_dlut_len;
static deadband_cal_state_t s_state;
static uint8_t s_outlier_seen;
static deadband_geo_sample_t s_geo_samples[M1_DEADBAND_GEO_SAMPLE_MAX];
static uint16_t s_geo_sample_count;

#if defined(M1_DEADBAND_GEO_DIFF_LOG_ENABLE) && (M1_DEADBAND_GEO_DIFF_LOG_ENABLE != 0)
static float s_geo_diff_leg_amps[M1_DEADBAND_LUT_MAX];
static float s_geo_diff_leg_vals[M1_DEADBAND_LUT_MAX];
#endif

#if (defined(M1_DEADBAND_LUT_COMMIT_NORMALIZE) && (M1_DEADBAND_LUT_COMMIT_NORMALIZE != 0)) || \
    (defined(M1_DEADBAND_LUT_RUNTIME_SCALE_AUTO) && (M1_DEADBAND_LUT_RUNTIME_SCALE_AUTO != 0))
static float deadband_cal_commit_gain(const float *amps, const float *vals, uint8_t len)
{
    float i_typ;
    float v_table;
    float gain;

    if (amps == NULL || vals == NULL || len < 2u) {
        return 1.0f;
    }

    i_typ = M1_ID_CAL_IQ_PROBE_A * M1_ID_CAL_D_TO_PHASE_COS;
    v_table = deadband_cal_lut_interp_v(amps, vals, len, i_typ);
    if (v_table < 0.05f) {
        return 1.0f;
    }

    gain = M1_DEADBAND_V_COMP_V / v_table;
    if (gain < 0.05f) {
        gain = 0.05f;
    } else if (gain > 1.0f) {
        gain = 1.0f;
    }
    return gain;
}
#endif

#if defined(M1_DEADBAND_LUT_RUNTIME_SCALE_AUTO) && (M1_DEADBAND_LUT_RUNTIME_SCALE_AUTO != 0)
static float deadband_cal_compute_auto_scale(const float *amps, const float *vals,
                                               uint8_t len)
{
    return deadband_cal_commit_gain(amps, vals, len);
}
#endif

#if defined(M1_DEADBAND_LUT_COMMIT_NORMALIZE) && (M1_DEADBAND_LUT_COMMIT_NORMALIZE != 0)
static void deadband_cal_normalize_plut_vals(float *vals, uint8_t len, float gain)
{
    uint8_t k;

    if (vals == NULL || len < 2u) {
        return;
    }

    for (k = 0u; k < len; k++) {
        vals[k] *= gain;
    }
}

static void deadband_cal_commit_normalize_pluts(void)
{
    const float *ref_amps;
    const float *ref_vals;
    float gain;
    uint8_t ph;

    if (s_dlut_len < 2u) {
        return;
    }

#if M1_DEADBAND_GEO_TWO_CLUSTER_ENABLE
    if (s_plut_cluster_valid != 0u) {
        ref_amps = s_plut_cluster_amps[0];
        ref_vals = s_plut_cluster_vals[0];
    } else
#endif
    {
        ref_amps = s_plut_amps[0];
        ref_vals = s_plut_vals[0];
    }

    gain = deadband_cal_commit_gain(ref_amps, ref_vals, s_dlut_len);

#if M1_DEADBAND_GEO_TWO_CLUSTER_ENABLE
    if (s_plut_cluster_valid != 0u) {
        deadband_cal_normalize_plut_vals(s_plut_cluster_vals[0], s_dlut_len, gain);
        deadband_cal_normalize_plut_vals(s_plut_cluster_vals[1], s_dlut_len, gain);
    }
#endif
    if (s_plut_triplet != 0u) {
        for (ph = 0u; ph < 3u; ph++) {
            deadband_cal_normalize_plut_vals(s_plut_vals[ph], s_dlut_len, gain);
        }
    } else if (s_plut_cluster_valid == 0u) {
        deadband_cal_normalize_plut_vals(s_plut_vals[0], s_dlut_len, gain);
    }

    deadband_set_lut_runtime_scale(1.0f);
}
#endif

static bool deadband_cal_export_nvm_sweep(factory_nvm_deadband_t *out)
{
    const float *a0;
    const float *v0;
    const float *a1;
    const float *v1;
    uint8_t len;
    uint8_t two_cluster;

    if (out == NULL) {
        return false;
    }

    len = s_dlut_len;
    if (len < 2u) {
        return false;
    }

#if M1_DEADBAND_GEO_TWO_CLUSTER_ENABLE
    if (s_plut_cluster_valid != 0u) {
        a0 = s_plut_cluster_amps[0];
        v0 = s_plut_cluster_vals[0];
        a1 = s_plut_cluster_amps[1];
        v1 = s_plut_cluster_vals[1];
        two_cluster = 1u;
    } else
#endif
    {
        a0 = s_plut_amps[0];
        v0 = s_plut_vals[0];
        a1 = s_plut_amps[0];
        v1 = s_plut_vals[0];
        two_cluster = 0u;
    }

    deadband_cal_pack_nvm_record(out, a0, v0, a1, v1, len, two_cluster);
    return true;
}

static void deadband_cal_build_phase_lut(void)
{
    uint8_t i;

#if M1_ID_CAL_FIX_THETA_ENABLE
    const float k = M1_ID_CAL_D_TO_PHASE_COS;

    for (i = 0u; i < s_dlut_len; i++) {
        s_plut_amps[0][i] = s_dlut_amps[i] * k;
        s_plut_vals[0][i] = s_dlut_vals[i] * k;
    }
#else
    for (i = 0u; i < s_dlut_len; i++) {
        s_plut_amps[0][i] = s_dlut_amps[i];
        s_plut_vals[0][i] = s_dlut_vals[i];
    }
#endif
}

#if M1_DEADBAND_LUT_LOW_FLAT_ENABLE

static float deadband_cal_dlut_interp_v(float id_amp)
{
    uint8_t i;

    if (s_dlut_len < 2u) {
        return 0.0f;
    }
    if (id_amp <= s_dlut_amps[0]) {
        return s_dlut_vals[0];
    }
    if (id_amp >= s_dlut_amps[s_dlut_len - 1u]) {
        return s_dlut_vals[s_dlut_len - 1u];
    }
    for (i = 0u; i < (s_dlut_len - 1u); i++) {
        float a0 = s_dlut_amps[i];
        float a1 = s_dlut_amps[i + 1u];

        if (id_amp <= a1) {
            float t = (id_amp - a0) / (a1 - a0);
            return s_dlut_vals[i] + t * (s_dlut_vals[i + 1u] - s_dlut_vals[i]);
        }
    }
    return s_dlut_vals[s_dlut_len - 1u];
}

/** Id∈[首点, LOW_FLAT_ID_A]：d 表 val 线性化（去掉 0.15~0.2 A 陡升），再 ×0.866 得 phase 表 */
static void deadband_cal_flatten_low_dlut(void)
{
    const float id_hi = M1_DEADBAND_LUT_LOW_FLAT_ID_A;
    const float id_lo = s_dlut_amps[0];
    float v_lo;
    float v_hi;
    uint8_t i;

    if (s_dlut_len < 2u || id_hi <= id_lo) {
        return;
    }

    v_lo = s_dlut_vals[0];
    v_hi = deadband_cal_dlut_interp_v(id_hi);

    for (i = 0u; i < s_dlut_len; i++) {
        if (s_dlut_amps[i] <= id_hi) {
            float t = (s_dlut_amps[i] - id_lo) / (id_hi - id_lo);
            s_dlut_vals[i] = v_lo + t * (v_hi - v_lo);
        }
    }
}

#endif /* M1_DEADBAND_LUT_LOW_FLAT_ENABLE */

#if M1_ID_CAL_LUT_DEDUP_ENABLE

/** 小数组 val 中位数（插入排序，≤8 点） */
static float deadband_cal_median_val(float *vals, uint8_t n)
{
    float tmp[M1_DEADBAND_LUT_MAX];
    uint8_t i;
    uint8_t j;

    if (n == 0u) {
        return 0.0f;
    }
    if (n > M1_DEADBAND_LUT_MAX) {
        n = M1_DEADBAND_LUT_MAX;
    }
    for (i = 0u; i < n; i++) {
        tmp[i] = vals[i];
    }
    for (i = 1u; i < n; i++) {
        float key = tmp[i];
        j = i;
        while (j > 0u && tmp[j - 1u] > key) {
            tmp[j] = tmp[j - 1u];
            j--;
        }
        tmp[j] = key;
    }
    return tmp[n / 2u];
}

#endif /* M1_ID_CAL_LUT_DEDUP_ENABLE */

/** Phase 2：amps 升序（插值安全） */
static void deadband_cal_sort_dlut(void)
{
    uint8_t n = s_dlut_len;
    uint8_t i;
    uint8_t j;

    if (n < 2u) {
        return;
    }

    for (i = 0u; i < n; i++) {
        for (j = 0u; j + 1u < n; j++) {
            if (s_dlut_amps[j] > s_dlut_amps[j + 1u]) {
                float ta = s_dlut_amps[j];
                float tv = s_dlut_vals[j];
                s_dlut_amps[j] = s_dlut_amps[j + 1u];
                s_dlut_vals[j] = s_dlut_vals[j + 1u];
                s_dlut_amps[j + 1u] = ta;
                s_dlut_vals[j + 1u] = tv;
            }
        }
    }
}

#if M1_ID_CAL_LUT_DEDUP_ENABLE

/** 近邻 amp 合并（val 中位数）；须已 sort */
static void deadband_cal_dedupe_dlut(void)
{
    uint8_t n = s_dlut_len;
    uint8_t write = 0u;
    uint8_t read = 0u;

    if (n < 2u) {
        return;
    }

    while (read < n) {
        uint8_t group_start = read;
        float group_vals[M1_DEADBAND_LUT_MAX];
        uint8_t group_n = 0u;

        while (read < n &&
               (read == group_start ||
                (s_dlut_amps[read] - s_dlut_amps[group_start]) <=
                    M1_ID_CAL_LUT_DEDUP_AMP_EPS_A)) {
            if (group_n < M1_DEADBAND_LUT_MAX) {
                group_vals[group_n] = s_dlut_vals[read];
                group_n++;
            }
            read++;
        }

        s_dlut_amps[write] = s_dlut_amps[group_start];
        s_dlut_vals[write] = deadband_cal_median_val(group_vals, group_n);
        write++;
    }
    s_dlut_len = write;
}

#endif /* M1_ID_CAL_LUT_DEDUP_ENABLE */

void deadband_cal_switch_runtime_lut(uint8_t use_d_table)
{
    if (s_dlut_len < 2u) {
        return;
    }

    if (use_d_table) {
        deadband_set_lut(s_dlut_amps, s_dlut_vals, s_dlut_len);
        deadband_set_lut_domain(1u);
        deadband_set_runtime_apply_ud(1u);
    } else {
        if (s_plut_triplet != 0u) {
            deadband_set_phase_luts(s_plut_amps[0], s_plut_vals[0],
                                    s_plut_amps[1], s_plut_vals[1],
                                    s_plut_amps[2], s_plut_vals[2],
                                    s_dlut_len);
        } else {
            deadband_set_lut(s_plut_amps[0], s_plut_vals[0], s_dlut_len);
        }
        deadband_set_lut_domain(0u);
        deadband_set_runtime_apply_ud(0u);
    }
    deadband_set_geo_dlut(s_dlut_amps, s_dlut_vals, s_dlut_len);
#if M1_DEADBAND_GEO_TWO_CLUSTER_ENABLE
    if (s_plut_cluster_valid != 0u) {
        deadband_set_cluster_luts(s_plut_cluster_amps[0], s_plut_cluster_vals[0],
                                  s_plut_cluster_amps[1], s_plut_cluster_vals[1],
                                  s_dlut_len);
    }
#endif
    deadband_set_mode(M1_DEADBAND_MODE_LUT);
}

void deadband_cal_reset(void)
{
    s_dlut_len = 0u;
    s_plut_triplet = 0u;
    s_plut_cluster_valid = 0u;
    s_state = DEADBAND_CAL_IDLE;
    s_outlier_seen = 0u;
    s_geo_sample_count = 0u;
    deadband_geo_reset();
}

static void deadband_cal_geo_push(float theta_el, float id_amp, float ud_val)
{
    deadband_geo_sample_t pts[3];
    uint8_t p;

    deadband_geo_dlut_point(theta_el, id_amp, ud_val, pts);
    for (p = 0u; p < 3u; p++) {
        if (M1_ID_CAL_GEO_U_MIN_V > 0.0f &&
            fabsf(theta_el - M1_ID_CAL_THETA_PASS0_B_RAD) < M1_ID_CAL_THETA_MATCH_RAD &&
            pts[p].u_abs < M1_ID_CAL_GEO_U_MIN_V) {
            continue;
        }
#if M1_DEADBAND_I_ZERO_DISABLE
        if (pts[p].i_abs <= 0.0f) {
            continue;
        }
#else
        if (pts[p].i_abs < M1_DEADBAND_I_ZERO_A) {
            continue;
        }
#endif
        if (s_geo_sample_count >= M1_DEADBAND_GEO_SAMPLE_MAX) {
            return;
        }
        pts[p].id_capture = id_amp;
        pts[p].theta_el = theta_el;
        s_geo_samples[s_geo_sample_count] = pts[p];
        s_geo_sample_count++;
    }
}

bool deadband_cal_capture_at(float id_a, float ud_pi_v, float id_ref_a,
                             float theta_el, uint8_t append_dlut)
{
    float id_amp;
    float ud_res;
    float ud_val;

    if (id_ref_a < M1_ID_CAL_I_MIN_A) {
        return false;
    }

    if (fabsf(id_a - id_ref_a) > M1_ID_CAL_CAPTURE_EPS_A) {
        return false;
    }

    id_amp = fabsf(id_a);
#if !M1_DEADBAND_I_ZERO_DISABLE
    if (id_amp < M1_DEADBAND_I_ZERO_A) {
        return false;
    }
#endif
    if (id_amp < M1_ID_CAL_I_MIN_A) {
        return false;
    }

    ud_res = ud_pi_v - id_a * g_m1_motor_cfg.rs_ohm;
    ud_val = fabsf(ud_res);

    if (ud_val > M1_ID_CAL_OUTLIER_V) {
        s_outlier_seen = 1u;
    }

    deadband_cal_geo_push(theta_el, id_amp, ud_val);

    if (!append_dlut) {
        s_state = DEADBAND_CAL_SWEEP;
        return true;
    }

    if (s_dlut_len >= M1_DEADBAND_LUT_MAX) {
        return false;
    }

    s_dlut_amps[s_dlut_len] = id_amp;
    s_dlut_vals[s_dlut_len] = ud_val;
    s_dlut_len++;
    s_state = DEADBAND_CAL_SWEEP;
    return true;
}

bool deadband_cal_capture(float id_a, float ud_pi_v, float id_ref_a)
{
    return deadband_cal_capture_at(id_a, ud_pi_v, id_ref_a,
                                   M1_ID_CAL_THETA_EL_RAD, 1u);
}

void deadband_cal_commit(void)
{
#if M1_ID_CAL_COMMIT_LUT
    if (s_dlut_len >= 2u) {
        deadband_cal_sort_dlut();
#if M1_ID_CAL_LUT_DEDUP_ENABLE
        deadband_cal_dedupe_dlut();
#endif
#if M1_DEADBAND_LUT_LOW_FLAT_ENABLE
        deadband_cal_flatten_low_dlut();
#endif
#if defined(M1_DEADBAND_GEO_BUILD_ENABLE) && (M1_DEADBAND_GEO_BUILD_ENABLE != 0)
#if M1_ID_CAL_DUAL_ANGLE_ENABLE
        if (s_geo_sample_count >= 2u) {
            uint8_t plut_len = 0u;

#if M1_DEADBAND_GEO_TRIPLET_ENABLE
            plut_len = deadband_geo_build_plut_triplet(s_geo_samples,
                                                       s_geo_sample_count,
                                                       s_dlut_amps, s_dlut_vals,
                                                       s_dlut_len,
                                                       s_plut_amps, s_plut_vals);
            s_plut_triplet = (plut_len >= 2u) ? 1u : 0u;
#endif
            if (plut_len < 2u) {
                plut_len = deadband_geo_build_plut(s_geo_samples,
                                                   s_geo_sample_count,
                                                   s_dlut_amps, s_dlut_vals,
                                                   s_dlut_len,
                                                   s_plut_amps[0], s_plut_vals[0]);
                s_plut_triplet = 0u;
            }
            if (plut_len < 2u) {
                deadband_geo_build_plut_from_dlut_30(s_dlut_amps, s_dlut_vals,
                                                     s_dlut_len,
                                                     s_plut_amps[0],
                                                     s_plut_vals[0]);
                s_plut_triplet = 0u;
            }
#if M1_DEADBAND_GEO_TWO_CLUSTER_ENABLE
            {
                uint8_t cl;

                for (cl = 0u; cl < 2u; cl++) {
                    if (deadband_geo_build_plut_cluster(s_geo_samples,
                                                        s_geo_sample_count,
                                                        s_dlut_amps, s_dlut_vals,
                                                        s_dlut_len, cl,
                                                        s_plut_cluster_amps[cl],
                                                        s_plut_cluster_vals[cl]) < 2u) {
                        deadband_geo_build_plut_from_dlut_30(
                            s_dlut_amps, s_dlut_vals, s_dlut_len,
                            s_plut_cluster_amps[cl], s_plut_cluster_vals[cl]);
                    }
                }
                s_plut_cluster_valid = 1u;
            }
#endif
        } else {
            deadband_geo_build_plut_from_dlut_30(s_dlut_amps, s_dlut_vals, s_dlut_len,
                                                 s_plut_amps[0], s_plut_vals[0]);
            s_plut_triplet = 0u;
#if M1_DEADBAND_GEO_TWO_CLUSTER_ENABLE
            {
                uint8_t cl;

                for (cl = 0u; cl < 2u; cl++) {
                    deadband_geo_build_plut_from_dlut_30(
                        s_dlut_amps, s_dlut_vals, s_dlut_len,
                        s_plut_cluster_amps[cl], s_plut_cluster_vals[cl]);
                }
                s_plut_cluster_valid = 1u;
            }
#endif
        }
#if defined(M1_DEADBAND_GEO_DIFF_LOG_ENABLE) && (M1_DEADBAND_GEO_DIFF_LOG_ENABLE != 0)
        {
            uint8_t i;
            const float k = M1_ID_CAL_D_TO_PHASE_COS;

            for (i = 0u; i < s_dlut_len; i++) {
                s_geo_diff_leg_amps[i] = s_dlut_amps[i] * k;
                s_geo_diff_leg_vals[i] = s_dlut_vals[i] * k;
            }
            deadband_geo_diff_update(s_plut_amps[0], s_plut_vals[0],
                                     s_geo_diff_leg_amps, s_geo_diff_leg_vals,
                                     s_dlut_len);
        }
#endif
#else
        deadband_geo_build_plut_from_dlut_30(s_dlut_amps, s_dlut_vals, s_dlut_len,
                                             s_plut_amps[0], s_plut_vals[0]);
        s_plut_triplet = 0u;
#endif
#else
        deadband_cal_build_phase_lut();
        s_plut_triplet = 0u;
#if defined(M1_DEADBAND_GEO_DIFF_LOG_ENABLE) && (M1_DEADBAND_GEO_DIFF_LOG_ENABLE != 0)
        {
            deadband_geo_build_plut_from_dlut_30(s_dlut_amps, s_dlut_vals, s_dlut_len,
                                                 s_geo_diff_leg_amps, s_geo_diff_leg_vals);
            deadband_geo_diff_update(s_geo_diff_leg_amps, s_geo_diff_leg_vals,
                                     s_plut_amps[0], s_plut_vals[0], s_dlut_len);
        }
#endif
#endif
        deadband_set_geo_dlut(s_dlut_amps, s_dlut_vals, s_dlut_len);
#if defined(M1_DEADBAND_LUT_COMMIT_NORMALIZE) && (M1_DEADBAND_LUT_COMMIT_NORMALIZE != 0)
        deadband_cal_commit_normalize_pluts();
#elif defined(M1_DEADBAND_LUT_RUNTIME_SCALE_AUTO) && (M1_DEADBAND_LUT_RUNTIME_SCALE_AUTO != 0)
        {
            const float *scale_amps = s_plut_amps[0];
            const float *scale_vals = s_plut_vals[0];

#if M1_DEADBAND_GEO_TWO_CLUSTER_ENABLE
            if (s_plut_cluster_valid != 0u) {
                scale_amps = s_plut_cluster_amps[0];
                scale_vals = s_plut_cluster_vals[0];
            }
#endif
            deadband_set_lut_runtime_scale(
                deadband_cal_compute_auto_scale(scale_amps, scale_vals, s_dlut_len));
        }
#endif
#if M1_DEADBAND_GEO_TWO_CLUSTER_ENABLE
        if (s_plut_cluster_valid != 0u) {
            deadband_set_cluster_luts(s_plut_cluster_amps[0], s_plut_cluster_vals[0],
                                      s_plut_cluster_amps[1], s_plut_cluster_vals[1],
                                      s_dlut_len);
        }
#endif
#if M1_ID_CAL_PASS1_USE_APPLY_DUTY
        deadband_cal_switch_runtime_lut(0u);
#elif M1_ID_CAL_LUT_VERIFY_SWEEP
        /* Pass1：d 表 Ud 验 capture；Iq 段 enter_iq_probe_on 再 switch_runtime_lut(0) */
        deadband_cal_switch_runtime_lut(1u);
#else
        /* Pass0→Iq 探路：跳过 Pass1，commit 后直接 phase 表 + apply_duty */
        deadband_cal_switch_runtime_lut(0u);
#endif
#if defined(M1_DEADBAND_NVM_COMMIT_ENABLE) && (M1_DEADBAND_NVM_COMMIT_ENABLE != 0)
        {
            factory_nvm_deadband_t db;

            if (deadband_cal_export_nvm(&db)) {
                db.runtime_scale = deadband_get_lut_runtime_scale();
                (void)factory_nvm_write_deadband(&db);
            }
        }
#endif
    }
#endif
    s_state = DEADBAND_CAL_DONE;
}

void deadband_cal_finish_sweep(void)
{
    s_state = DEADBAND_CAL_DONE;
}

deadband_cal_state_t deadband_cal_get_state(void)
{
    return s_state;
}

uint8_t deadband_cal_len(void)
{
    return s_dlut_len;
}

bool deadband_cal_outlier_seen(void)
{
    return s_outlier_seen != 0u;
}

uint16_t deadband_cal_geo_sample_len(void)
{
    return s_geo_sample_count;
}

const float *deadband_cal_amps(void)
{
    return s_dlut_amps;
}

const float *deadband_cal_vals(void)
{
    return s_dlut_vals;
}

const float *deadband_cal_amps_phase(void)
{
    return s_plut_amps[0];
}

const float *deadband_cal_vals_phase(void)
{
    return s_plut_vals[0];
}

const float *deadband_cal_amps_phase_ph(uint8_t phase)
{
    if (phase > 2u) {
        return NULL;
    }
    return s_plut_amps[phase];
}

const float *deadband_cal_vals_phase_ph(uint8_t phase)
{
    if (phase > 2u) {
        return NULL;
    }
    return s_plut_vals[phase];
}

uint8_t deadband_cal_phase_lut_triplet(void)
{
    return s_plut_triplet;
}

#else /* M1_ID_LOCK_CAL_SWEEP */

void deadband_cal_reset(void)
{
}

bool deadband_cal_capture(float id_a, float ud_pi_v, float id_ref_a)
{
    (void)id_a;
    (void)ud_pi_v;
    (void)id_ref_a;
    return false;
}

bool deadband_cal_capture_at(float id_a, float ud_pi_v, float id_ref_a,
                             float theta_el, uint8_t append_dlut)
{
    (void)id_a;
    (void)ud_pi_v;
    (void)id_ref_a;
    (void)theta_el;
    (void)append_dlut;
    return false;
}

void deadband_cal_commit(void)
{
}

void deadband_cal_switch_runtime_lut(uint8_t use_d_table)
{
    (void)use_d_table;
}

void deadband_cal_finish_sweep(void)
{
}

deadband_cal_state_t deadband_cal_get_state(void)
{
    return DEADBAND_CAL_IDLE;
}

uint8_t deadband_cal_len(void)
{
    return 0u;
}

bool deadband_cal_outlier_seen(void)
{
    return false;
}

uint16_t deadband_cal_geo_sample_len(void)
{
    return 0u;
}

const float *deadband_cal_amps(void)
{
    return NULL;
}

const float *deadband_cal_vals(void)
{
    return NULL;
}

const float *deadband_cal_amps_phase(void)
{
    return NULL;
}

const float *deadband_cal_vals_phase(void)
{
    return NULL;
}

const float *deadband_cal_amps_phase_ph(uint8_t phase)
{
    (void)phase;
    return NULL;
}

const float *deadband_cal_vals_phase_ph(uint8_t phase)
{
    (void)phase;
    return NULL;
}

uint8_t deadband_cal_phase_lut_triplet(void)
{
    return 0u;
}

#endif /* M1_ID_LOCK_CAL_SWEEP */
