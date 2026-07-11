/**
 * @file deadband_geo.c
 * @brief 论文 §4.4 电压域反 Park/Clarke 与 phase LUT 几何建表。
 */

#include "deadband_geo.h"

#include <stddef.h>

#include "deadband.h"
#include "motor_params_m1.h"
#include "foc_svpwm.h"

#ifndef M1_ID_CAL_D_TO_PHASE_COS
#define M1_ID_CAL_D_TO_PHASE_COS     0.8660254037844386f
#endif

#ifndef M1_ID_CAL_THETA_EL_RAD
#define M1_ID_CAL_THETA_EL_RAD       0.5235987755982988f
#endif

#define DEADBAND_GEO_HALF_F            0.5f
#define DEADBAND_GEO_SQRT3_2           0.8660254037844386f

#if defined(M1_DEADBAND_GEO_DIFF_LOG_ENABLE) && (M1_DEADBAND_GEO_DIFF_LOG_ENABLE != 0)
static float s_geo_diff_amp_max;
static float s_geo_diff_val_max;
#endif

#ifndef M1_ID_CAL_CAPTURE_EPS_A
#define M1_ID_CAL_CAPTURE_EPS_A    0.03f
#endif

#define DEADBAND_GEO_MERGE_U_MAX     6u

static float deadband_geo_fabsf(float x)
{
    return (x >= 0.0f) ? x : -x;
}

#ifndef M1_ID_CAL_THETA_PASS0_A_RAD
#define M1_ID_CAL_THETA_PASS0_A_RAD  M1_ID_CAL_THETA_EL_RAD
#endif

#ifndef M1_ID_CAL_THETA_MATCH_RAD
#define M1_ID_CAL_THETA_MATCH_RAD    0.02f
#endif

/** Pass0-A（30°）样本，用于 merge amp 定标（式 4-21） */
static uint8_t deadband_geo_theta_is_pass0_a(float theta_el)
{
    return (deadband_geo_fabsf(theta_el - M1_ID_CAL_THETA_PASS0_A_RAD) <
            M1_ID_CAL_THETA_MATCH_RAD) ?
           1u :
           0u;
}

static uint8_t deadband_geo_theta_is_pass0_b(float theta_el)
{
    return (deadband_geo_fabsf(theta_el - M1_ID_CAL_THETA_PASS0_B_RAD) <
            M1_ID_CAL_THETA_MATCH_RAD) ?
           1u :
           0u;
}

static uint8_t deadband_geo_theta_matches_cluster(float theta_el, uint8_t cluster)
{
    if (cluster == 0u) {
        return deadband_geo_theta_is_pass0_a(theta_el);
    }
    return deadband_geo_theta_is_pass0_b(theta_el);
}

uint8_t deadband_geo_theta_cluster_idx(float theta_el)
{
    const float pi3 = 1.0471975511965976f;
    const float pi6 = 0.5235987755982988f;
    const float two_pi = 6.28318530718f;
    float t = theta_el;

    while (t < 0.0f) {
        t += two_pi;
    }
    while (t >= pi3) {
        t -= pi3;
    }
    return (t < pi6) ? 1u : 0u;
}

static void deadband_geo_inv_clarke(float u_alpha, float u_beta,
                                    float *ua, float *ub, float *uc)
{
    *ua = u_alpha;
    *ub = (-DEADBAND_GEO_HALF_F * u_alpha) + (DEADBAND_GEO_SQRT3_2 * u_beta);
    *uc = (-DEADBAND_GEO_HALF_F * u_alpha) - (DEADBAND_GEO_SQRT3_2 * u_beta);
}

static void deadband_geo_id_to_abc(float id, float theta_el,
                                   float *ia, float *ib, float *ic)
{
    float i_alpha;
    float i_beta;

    Anti_Park_Transform(id, 0.0f, theta_el, &i_alpha, &i_beta);
    deadband_geo_inv_clarke(i_alpha, i_beta, ia, ib, ic);
}

static void deadband_geo_sort_samples(deadband_geo_sample_t *samples, uint16_t n)
{
    uint16_t i;
    uint16_t j;

    for (i = 1u; i < n; i++) {
        deadband_geo_sample_t key = samples[i];

        j = i;
        while (j > 0u && samples[j - 1u].i_abs > key.i_abs) {
            samples[j] = samples[j - 1u];
            j--;
        }
        samples[j] = key;
    }
}

#ifndef M1_ID_CAL_LUT_DEDUP_AMP_EPS_A
#define M1_ID_CAL_LUT_DEDUP_AMP_EPS_A  0.005f
#endif

static float deadband_geo_median_u(float *vals, uint8_t n)
{
    uint8_t i;
    uint8_t j;

    if (n == 0u) {
        return 0.0f;
    }
    for (i = 1u; i < n; i++) {
        float key = vals[i];

        j = i;
        while (j > 0u && vals[j - 1u] > key) {
            vals[j] = vals[j - 1u];
            j--;
        }
        vals[j] = key;
    }
    return vals[n / 2u];
}

/** 按 |i| 近邻合并，u 取中位数（双角多样本池） */
static uint16_t deadband_geo_dedupe_samples(deadband_geo_sample_t *samples, uint16_t n)
{
    const float eps = M1_ID_CAL_LUT_DEDUP_AMP_EPS_A;
    uint16_t write = 0u;
    uint16_t read = 0u;

    if (n < 2u) {
        return n;
    }

    while (read < n) {
        uint16_t group_start = read;
        float group_u[8];
        uint8_t group_n = 0u;

        while (read < n &&
               (read == group_start ||
                (samples[read].i_abs - samples[group_start].i_abs) <= eps)) {
            if (group_n < 8u) {
                group_u[group_n] = samples[read].u_abs;
                group_n++;
            }
            read++;
        }

        samples[write].i_abs = samples[group_start].i_abs;
        samples[write].u_abs = deadband_geo_median_u(group_u, group_n);
        samples[write].phase = samples[group_start].phase;
        write++;
    }

    return write;
}

static float deadband_geo_interp_u(const deadband_geo_sample_t *samples, uint16_t n,
                                   float i_abs)
{
    uint16_t i;

    if (n == 0u) {
        return 0.0f;
    }
    if (i_abs <= samples[0].i_abs) {
        return samples[0].u_abs;
    }
    if (i_abs >= samples[n - 1u].i_abs) {
        return samples[n - 1u].u_abs;
    }

    for (i = 0u; i < (n - 1u); i++) {
        float i0 = samples[i].i_abs;
        float i1 = samples[i + 1u].i_abs;

        if (i_abs <= i1) {
            float t = (i_abs - i0) / (i1 - i0);
            return samples[i].u_abs + t * (samples[i + 1u].u_abs - samples[i].u_abs);
        }
    }

    return samples[n - 1u].u_abs;
}

void deadband_geo_reset(void)
{
#if defined(M1_DEADBAND_GEO_DIFF_LOG_ENABLE) && (M1_DEADBAND_GEO_DIFF_LOG_ENABLE != 0)
    s_geo_diff_amp_max = 0.0f;
    s_geo_diff_val_max = 0.0f;
#endif
}

void deadband_geo_ud_to_abc(float ud_res, float theta_el,
                            float *ua, float *ub, float *uc)
{
    float u_alpha;
    float u_beta;

    if (ua == NULL || ub == NULL || uc == NULL) {
        return;
    }

    Anti_Park_Transform(ud_res, 0.0f, theta_el, &u_alpha, &u_beta);
    deadband_geo_inv_clarke(u_alpha, u_beta, ua, ub, uc);
}

void deadband_geo_remove_u0(float *ua, float *ub, float *uc)
{
    float u0;

    if (ua == NULL || ub == NULL || uc == NULL) {
        return;
    }

    u0 = (*ua + *ub + *uc) / 3.0f;
    *ua -= u0;
    *ub -= u0;
    *uc -= u0;
}

void deadband_geo_dlut_point(float theta_el, float id, float ud_res,
                             deadband_geo_sample_t out[3])
{
    float ua;
    float ub;
    float uc;
    float ia;
    float ib;
    float ic;
    float id_abs = deadband_geo_fabsf(id);
    float ud_abs = deadband_geo_fabsf(ud_res);

    if (out == NULL) {
        return;
    }

    deadband_geo_ud_to_abc(ud_abs, theta_el, &ua, &ub, &uc);
    deadband_geo_remove_u0(&ua, &ub, &uc);
    deadband_geo_id_to_abc(id_abs, theta_el, &ia, &ib, &ic);

    out[0].phase = 0u;
    out[0].i_abs = deadband_geo_fabsf(ia);
    out[0].u_abs = deadband_geo_fabsf(ua);
    out[0].id_capture = id_abs;
    out[0].theta_el = theta_el;

    out[1].phase = 1u;
    out[1].i_abs = deadband_geo_fabsf(ib);
    out[1].u_abs = deadband_geo_fabsf(ub);
    out[1].id_capture = id_abs;
    out[1].theta_el = theta_el;

    out[2].phase = 2u;
    out[2].i_abs = deadband_geo_fabsf(ic);
    out[2].u_abs = deadband_geo_fabsf(uc);
    out[2].id_capture = id_abs;
    out[2].theta_el = theta_el;
}

void deadband_geo_build_plut_from_dlut_30(const float *dlut_amps, const float *dlut_vals,
                                          uint8_t dlut_len,
                                          float *plut_amps, float *plut_vals)
{
    uint8_t i;
    const float theta_el = M1_ID_CAL_THETA_EL_RAD;

    if (dlut_amps == NULL || dlut_vals == NULL ||
        plut_amps == NULL || plut_vals == NULL || dlut_len == 0u) {
        return;
    }

    (void)theta_el;

    for (i = 0u; i < dlut_len; i++) {
        deadband_geo_sample_t samples[3];
        uint8_t p;
        uint8_t best = 0u;

        deadband_geo_dlut_point(theta_el, dlut_amps[i], dlut_vals[i], samples);
        for (p = 1u; p < 3u; p++) {
            if (samples[p].i_abs > samples[best].i_abs) {
                best = p;
            }
        }

        plut_amps[i] = samples[best].i_abs;
        plut_vals[i] = samples[best].u_abs;
    }
}

static void deadband_geo_fallback_30(float id_amp, float ud_res,
                                     float *i_out, float *u_out)
{
    deadband_geo_sample_t pts[3];
    float u_pool[3];
    uint8_t p;
    uint8_t u_n = 0u;
    uint8_t best = 0u;

    deadband_geo_dlut_point(M1_ID_CAL_THETA_PASS0_A_RAD, id_amp, ud_res, pts);
    for (p = 0u; p < 3u; p++) {
        if (pts[p].i_abs > pts[best].i_abs) {
            best = p;
        }
        if (u_n < 3u) {
            u_pool[u_n] = pts[p].u_abs;
            u_n++;
        }
    }

    *i_out = pts[best].i_abs;
    if (u_n > 0u) {
        *u_out = deadband_geo_median_u(u_pool, u_n);
    } else {
        *u_out = pts[best].u_abs;
    }
}

/**
 * 单档 Id_k（proposed merge）：
 * amp = Pass0-A（30°）max|i|；
 * val = MERGE_VAL30_ONLY ? 仅 30° |u'| median : 双角 |u'| median。
 */
static uint8_t deadband_geo_merge_id_bin(const deadband_geo_sample_t *samples,
                                         uint16_t n, float id_k,
                                         float *amp_out, float *val_out)
{
    float u_pool[DEADBAND_GEO_MERGE_U_MAX];
    uint8_t u_n = 0u;
    uint16_t i;
    float amp_max = 0.0f;

    if (amp_out == NULL || val_out == NULL) {
        return 0u;
    }

    for (i = 0u; i < n; i++) {
        if (deadband_geo_fabsf(samples[i].id_capture - id_k) > M1_ID_CAL_CAPTURE_EPS_A) {
            continue;
        }

        if (deadband_geo_theta_is_pass0_a(samples[i].theta_el)) {
            if (samples[i].i_abs > amp_max) {
                amp_max = samples[i].i_abs;
            }
#if M1_DEADBAND_GEO_MERGE_VAL30_ONLY
            if (u_n < DEADBAND_GEO_MERGE_U_MAX) {
                u_pool[u_n] = samples[i].u_abs;
                u_n++;
            }
#endif
        }
#if !M1_DEADBAND_GEO_MERGE_VAL30_ONLY
        if (u_n < DEADBAND_GEO_MERGE_U_MAX) {
            u_pool[u_n] = samples[i].u_abs;
            u_n++;
        }
#endif
    }

    if (amp_max <= 0.0f || u_n == 0u) {
        return 0u;
    }

    *amp_out = amp_max;
    *val_out = deadband_geo_median_u(u_pool, u_n);
    return 1u;
}

/**
 * 单档 Id_k、单簇：amp=该簇 max|i|；val=该簇 |u'| median。
 */
static uint8_t deadband_geo_merge_id_bin_cluster(const deadband_geo_sample_t *samples,
                                                 uint16_t n, float id_k,
                                                 uint8_t cluster,
                                                 float *amp_out, float *val_out)
{
    float u_pool[DEADBAND_GEO_MERGE_U_MAX];
    uint8_t u_n = 0u;
    uint16_t i;
    float amp_max = 0.0f;

    if (amp_out == NULL || val_out == NULL) {
        return 0u;
    }

    for (i = 0u; i < n; i++) {
        if (deadband_geo_fabsf(samples[i].id_capture - id_k) > M1_ID_CAL_CAPTURE_EPS_A) {
            continue;
        }
        if (!deadband_geo_theta_matches_cluster(samples[i].theta_el, cluster)) {
            continue;
        }

        if (samples[i].i_abs > amp_max) {
            amp_max = samples[i].i_abs;
        }
        if (u_n < DEADBAND_GEO_MERGE_U_MAX) {
            u_pool[u_n] = samples[i].u_abs;
            u_n++;
        }
    }

    if (amp_max <= 0.0f || u_n == 0u) {
        return 0u;
    }

    *amp_out = amp_max;
    *val_out = deadband_geo_median_u(u_pool, u_n);
    return 1u;
}

static void deadband_geo_sort_plut_pairs(float *amps, float *vals, uint8_t len)
{
    uint8_t i;
    uint8_t j;

    for (i = 1u; i < len; i++) {
        float key_a = amps[i];
        float key_v = vals[i];
        j = i;

        while (j > 0u && amps[j - 1u] > key_a) {
            amps[j] = amps[j - 1u];
            vals[j] = vals[j - 1u];
            j--;
        }
        amps[j] = key_a;
        vals[j] = key_v;
    }
}

/** 式(4-28) 建表后保证 f(|i|) 随 |i| 非降，便于 runtime 线性插值 */
static void deadband_geo_enforce_val_monotone(float *vals, uint8_t len)
{
    uint8_t i;

    for (i = 1u; i < len; i++) {
        if (vals[i] < vals[i - 1u]) {
            vals[i] = vals[i - 1u];
        }
    }
}

/** 合并 amp 近邻重复点（排序后），保留较大 val */
static uint8_t deadband_geo_dedupe_plut_pairs(float *amps, float *vals, uint8_t len)
{
    uint8_t w = 0u;
    uint8_t r;

    if (len < 2u) {
        return len;
    }

    for (r = 0u; r < len; r++) {
        if (w > 0u &&
            deadband_geo_fabsf(amps[r] - amps[w - 1u]) < M1_ID_CAL_LUT_DEDUP_AMP_EPS_A) {
            if (vals[r] > vals[w - 1u]) {
                vals[w - 1u] = vals[r];
            }
            continue;
        }
        amps[w] = amps[r];
        vals[w] = vals[r];
        w++;
    }
    return w;
}

/**
 * 单档 Id_k、单相：amp=Pass0-A（30°）该相 max|i|；val=双角该相 |u'| median。
 */
static uint8_t deadband_geo_merge_id_bin_phase(const deadband_geo_sample_t *samples,
                                               uint16_t n, float id_k,
                                               uint8_t phase,
                                               float *amp_out, float *val_out)
{
    float u_pool[DEADBAND_GEO_MERGE_U_MAX];
    uint8_t u_n = 0u;
    uint16_t i;
    float amp_max = 0.0f;
    float amp_any = 0.0f;

    if (amp_out == NULL || val_out == NULL) {
        return 0u;
    }

    for (i = 0u; i < n; i++) {
        if (samples[i].phase != phase) {
            continue;
        }
        if (deadband_geo_fabsf(samples[i].id_capture - id_k) > M1_ID_CAL_CAPTURE_EPS_A) {
            continue;
        }

        if (deadband_geo_theta_is_pass0_a(samples[i].theta_el) &&
            samples[i].i_abs > amp_max) {
            amp_max = samples[i].i_abs;
        }
        if (samples[i].i_abs > amp_any) {
            amp_any = samples[i].i_abs;
        }
#if M1_DEADBAND_GEO_MERGE_VAL30_ONLY
        if (deadband_geo_theta_is_pass0_a(samples[i].theta_el)) {
            if (u_n < DEADBAND_GEO_MERGE_U_MAX) {
                u_pool[u_n] = samples[i].u_abs;
                u_n++;
            }
        }
#else
        if (u_n < DEADBAND_GEO_MERGE_U_MAX) {
            u_pool[u_n] = samples[i].u_abs;
            u_n++;
        }
#endif
    }

    if (amp_max <= 0.0f) {
        amp_max = amp_any;
    }
    if (amp_max <= 0.0f || u_n == 0u) {
        return 0u;
    }

    *amp_out = amp_max;
    *val_out = deadband_geo_median_u(u_pool, u_n);
    return 1u;
}

static void deadband_geo_fallback_30_phase(uint8_t phase, float id_amp, float ud_res,
                                           float *i_out, float *u_out)
{
    deadband_geo_sample_t pts[3];

    deadband_geo_dlut_point(M1_ID_CAL_THETA_PASS0_A_RAD, id_amp, ud_res, pts);
    if (phase > 2u) {
        phase = 0u;
    }
    *i_out = pts[phase].i_abs;
    *u_out = pts[phase].u_abs;
}

uint8_t deadband_geo_build_plut_triplet(const deadband_geo_sample_t *samples,
                                        uint16_t n,
                                        const float *id_anchor,
                                        const float *ud_anchor,
                                        uint8_t anchor_len,
                                        float plut_amps[3][M1_DEADBAND_LUT_MAX],
                                        float plut_vals[3][M1_DEADBAND_LUT_MAX])
{
    uint8_t ph;
    uint8_t k;
    uint8_t out_len;

    if (samples == NULL || id_anchor == NULL || ud_anchor == NULL ||
        plut_amps == NULL || plut_vals == NULL ||
        n == 0u || anchor_len < 2u) {
        return 0u;
    }

    if (anchor_len > M1_DEADBAND_LUT_MAX) {
        anchor_len = M1_DEADBAND_LUT_MAX;
    }

    out_len = anchor_len;
    for (ph = 0u; ph < 3u; ph++) {
        for (k = 0u; k < out_len; k++) {
            const float id_k = id_anchor[k];

            if (!deadband_geo_merge_id_bin_phase(samples, n, id_k, ph,
                                                 &plut_amps[ph][k],
                                                 &plut_vals[ph][k])) {
                deadband_geo_fallback_30_phase(ph, id_k, ud_anchor[k],
                                               &plut_amps[ph][k],
                                               &plut_vals[ph][k]);
            }
        }
        deadband_geo_sort_plut_pairs(plut_amps[ph], plut_vals[ph], out_len);
        deadband_geo_enforce_val_monotone(plut_vals[ph], out_len);
    }

    return out_len;
}

uint8_t deadband_geo_build_plut(const deadband_geo_sample_t *samples, uint16_t n,
                                const float *id_anchor, const float *ud_anchor,
                                uint8_t anchor_len,
                                float *plut_amps, float *plut_vals)
{
    uint8_t k;
    uint8_t out_len;

    if (samples == NULL || id_anchor == NULL || ud_anchor == NULL ||
        plut_amps == NULL || plut_vals == NULL ||
        n == 0u || anchor_len < 2u) {
        return 0u;
    }

    if (anchor_len > M1_DEADBAND_LUT_MAX) {
        anchor_len = M1_DEADBAND_LUT_MAX;
    }

    out_len = anchor_len;
    for (k = 0u; k < out_len; k++) {
        const float id_k = id_anchor[k];

        if (!deadband_geo_merge_id_bin(samples, n, id_k,
                                       &plut_amps[k], &plut_vals[k])) {
            deadband_geo_fallback_30(id_k, ud_anchor[k],
                                   &plut_amps[k], &plut_vals[k]);
        }
    }

    deadband_geo_sort_plut_pairs(plut_amps, plut_vals, out_len);
    out_len = deadband_geo_dedupe_plut_pairs(plut_amps, plut_vals, out_len);
    deadband_geo_enforce_val_monotone(plut_vals, out_len);

    return out_len;
}

uint8_t deadband_geo_build_plut_cluster(const deadband_geo_sample_t *samples,
                                        uint16_t n,
                                        const float *id_anchor,
                                        const float *ud_anchor,
                                        uint8_t anchor_len,
                                        uint8_t cluster,
                                        float *plut_amps, float *plut_vals)
{
    uint8_t k;
    uint8_t out_len;

    if (samples == NULL || id_anchor == NULL || ud_anchor == NULL ||
        plut_amps == NULL || plut_vals == NULL ||
        n == 0u || anchor_len < 2u || cluster > 1u) {
        return 0u;
    }

    if (anchor_len > M1_DEADBAND_LUT_MAX) {
        anchor_len = M1_DEADBAND_LUT_MAX;
    }

    out_len = anchor_len;
    for (k = 0u; k < out_len; k++) {
        const float id_k = id_anchor[k];

        if (!deadband_geo_merge_id_bin_cluster(samples, n, id_k, cluster,
                                               &plut_amps[k], &plut_vals[k])) {
            deadband_geo_fallback_30(id_k, ud_anchor[k],
                                     &plut_amps[k], &plut_vals[k]);
        }
    }

    deadband_geo_sort_plut_pairs(plut_amps, plut_vals, out_len);
    out_len = deadband_geo_dedupe_plut_pairs(plut_amps, plut_vals, out_len);
    deadband_geo_enforce_val_monotone(plut_vals, out_len);

    return out_len;
}

#if defined(M1_DEADBAND_GEO_DIFF_LOG_ENABLE) && (M1_DEADBAND_GEO_DIFF_LOG_ENABLE != 0)

void deadband_geo_diff_update(const float *geo_amps, const float *geo_vals,
                              const float *ref_amps, const float *ref_vals,
                              uint8_t len)
{
    uint8_t i;

    s_geo_diff_amp_max = 0.0f;
    s_geo_diff_val_max = 0.0f;

    if (geo_amps == NULL || geo_vals == NULL ||
        ref_amps == NULL || ref_vals == NULL || len == 0u) {
        return;
    }

    for (i = 0u; i < len; i++) {
        float da = deadband_geo_fabsf(geo_amps[i] - ref_amps[i]);
        float dv = deadband_geo_fabsf(geo_vals[i] - ref_vals[i]);

        if (da > s_geo_diff_amp_max) {
            s_geo_diff_amp_max = da;
        }
        if (dv > s_geo_diff_val_max) {
            s_geo_diff_val_max = dv;
        }
    }
}

float deadband_geo_diff_amp_max(void)
{
    return s_geo_diff_amp_max;
}

float deadband_geo_diff_val_max(void)
{
    return s_geo_diff_val_max;
}

#endif /* M1_DEADBAND_GEO_DIFF_LOG_ENABLE */
