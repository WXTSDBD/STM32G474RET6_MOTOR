/**
 * @file deadband_geo.h
 * @date 2026-10-06
 * @brief 电压域反变换和相表几何建表。

 *
 * 建表在标定 commit 里调用，不进 20 kHz 热路径。
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
 */

#ifndef DEADBAND_GEO_H
#define DEADBAND_GEO_H

#include <stdint.h>

#include "deadband.h"
#include "motor_params_m1.h"

#ifndef M1_DEADBAND_GEO_DIFF_LOG_ENABLE
#define M1_DEADBAND_GEO_DIFF_LOG_ENABLE  0
#endif

#ifdef __cplusplus
extern "C" {
#endif

/**
 * 单条 abc 样本：建表阶段 (|i_phase|, |u'_phase|)，u' 已去 u₀。
 * id_capture = Pass0 档 |Id|；theta_el = 采样锁轴角（30° / 0°）。
 */
typedef struct {
    float i_abs;
    float u_abs;
    float id_capture;
    float theta_el;
    uint8_t phase;
} deadband_geo_sample_t;

void deadband_geo_reset(void);

/** 式(4-17)：ud=ud_res, uq=0 → 反 Park → 反 Clarke → ua,ub,uc (V) */
void deadband_geo_ud_to_abc(float ud_res, float theta_el,
                            float *ua, float *ub, float *uc);

/** 建表 u₀ 去除：u' = u − (ua+ub+uc)/3 */
void deadband_geo_remove_u0(float *ua, float *ub, float *uc);

/**
 * @brief 运行 θ 选簇：0=30° 族（π/6 mod π/3 上段），1=0° 族（下段）。
 *        对应 Pass0-A / Pass0-B 分簇 plut。
 */
uint8_t deadband_geo_theta_cluster_idx(float theta_el);

/**
 * @brief 由 (θe, Id, Ud_res) 推算三相 |i|、|u'|（Iq=0 锁轴标定）。
 * @param out 长度 3，phase 0=A … 2=C
 */
void deadband_geo_dlut_point(float theta_el, float id, float ud_res,
                             deadband_geo_sample_t out[3]);

/**
 * @brief 论文 ④⑤：按 udinv(Id_k) 档合并双角 abc 样本 → plut（式 4-28）。
 *
 * 每档 k：amp[k]=Pass0-A（30°）max|i|；val[k]=双角 |u'| 中位数（merge 不用 OUTLIER）。
 *
 * @param id_anchor  Pass0-A d 表 amps（=s_dlut_amps，已 sort）
 * @param ud_anchor  与 id_anchor 同序的 Ud_res；无样本时回退 30° geo
 * @return 写入 plut 点数；0 表示失败
 */
uint8_t deadband_geo_build_plut(const deadband_geo_sample_t *samples, uint16_t n,
                                const float *id_anchor, const float *ud_anchor,
                                uint8_t anchor_len,
                                float *plut_amps, float *plut_vals);

/**
 * @brief 单角簇建表：cluster 0=Pass0-A(30°)，1=Pass0-B(0°)；无样本时 30° geo 回退。
 */
uint8_t deadband_geo_build_plut_cluster(const deadband_geo_sample_t *samples,
                                        uint16_t n,
                                        const float *id_anchor,
                                        const float *ud_anchor,
                                        uint8_t anchor_len,
                                        uint8_t cluster,
                                        float *plut_amps, float *plut_vals);

/**
 * @brief 分相建表：phase 0/1/2 各 32 点，样本池按相分池拟合（双角）。
 * @param plut_amps  [3][len] 写入
 * @param plut_vals  [3][len] 写入
 * @return 每相点数（= anchor_len）；0 表示失败
 */
uint8_t deadband_geo_build_plut_triplet(const deadband_geo_sample_t *samples,
                                        uint16_t n,
                                        const float *id_anchor,
                                        const float *ud_anchor,
                                        uint8_t anchor_len,
                                        float plut_amps[3][M1_DEADBAND_LUT_MAX],
                                        float plut_vals[3][M1_DEADBAND_LUT_MAX]);

/**
 * @brief 30° 单角：由 d 表经 geo 路径生成 plut（应 ≈ × cos30°）。
 */
void deadband_geo_build_plut_from_dlut_30(const float *dlut_amps, const float *dlut_vals,
                                          uint8_t dlut_len,
                                          float *plut_amps, float *plut_vals);

#if defined(M1_DEADBAND_GEO_DIFF_LOG_ENABLE) && (M1_DEADBAND_GEO_DIFF_LOG_ENABLE != 0)
void deadband_geo_diff_update(const float *geo_amps, const float *geo_vals,
                              const float *ref_amps, const float *ref_vals,
                              uint8_t len);
float deadband_geo_diff_amp_max(void);
float deadband_geo_diff_val_max(void);
#endif

#ifdef __cplusplus
}
#endif

#endif /* DEADBAND_GEO_H */
