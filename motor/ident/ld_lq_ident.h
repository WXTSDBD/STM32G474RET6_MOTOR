/**
 * @file ld_lq_ident.h
 * @brief Pass0+Rs 后 VASI：MCU 存 coarse(500Hz)+fine(1kHz)[+f2] 或 fine-only；曲面 RLS 可离线。
 */

#ifndef LD_LQ_IDENT_H
#define LD_LQ_IDENT_H

#include <stdint.h>

#include "motor_params_m1.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef M1_LD_LQ_ID_BIAS_N
#define M1_LD_LQ_ID_BIAS_N  3u
#endif
#ifndef M1_LD_LQ_IQ_BIAS_N
#define M1_LD_LQ_IQ_BIAS_N  3u
#endif
#define M1_LD_LQ_GRID_N  ((uint8_t)(M1_LD_LQ_ID_BIAS_N * M1_LD_LQ_IQ_BIAS_N))

typedef struct {
    /** coarse @ M1_LD_LQ_IDENT_F_COARSE_HZ（默认 500 Hz） */
    float ld_h[M1_LD_LQ_GRID_N];
    float lq_h[M1_LD_LQ_GRID_N];
    /** fine @ M1_LD_LQ_IDENT_F_FINE_HZ（默认 1 kHz） */
    float ld_h_fine[M1_LD_LQ_GRID_N];
    float lq_h_fine[M1_LD_LQ_GRID_N];
#if M1_LD_LQ_IDENT_F2_ENABLE
    /** f2 @ M1_LD_LQ_IDENT_F_F2_HZ（默认 2 kHz，实验第三档） */
    float ld_h_f2[M1_LD_LQ_GRID_N];
    float lq_h_f2[M1_LD_LQ_GRID_N];
    uint8_t ld_valid_f2[M1_LD_LQ_GRID_N];
    uint8_t lq_valid_f2[M1_LD_LQ_GRID_N];
    uint16_t n_ld_ok_f2;
    uint16_t n_lq_ok_f2;
    uint8_t ok_f2;
#endif
    float id_bias[M1_LD_LQ_GRID_N];
    float iq_bias[M1_LD_LQ_GRID_N];
    uint8_t ld_valid[M1_LD_LQ_GRID_N];
    uint8_t lq_valid[M1_LD_LQ_GRID_N];
    uint8_t ld_valid_fine[M1_LD_LQ_GRID_N];
    uint8_t lq_valid_fine[M1_LD_LQ_GRID_N];
    uint16_t n_ld_ok;
    uint16_t n_lq_ok;
    uint16_t n_ld_ok_fine;
    uint16_t n_lq_ok_fine;
    float rs_used_ohm;
    uint8_t ok;
    uint8_t ok_fine;
} ld_lq_ident_result_t;

#if M1_LD_LQ_IDENT_ENABLE

void ld_lq_ident_init(void);
/** @param rs_ohm Rs 辨识结果；<=0 时用 M1_RS_OHM */
void ld_lq_ident_arm(float rs_ohm);
/** 首格点偏置（G0 Id=0.5 Iq=0），供 bumpless 切入 */
void ld_lq_ident_first_grid_bias(float *id0, float *iq0);
void ld_lq_ident_tick(float id_fb, float iq_fb, float ud_pi, float uq_pi,
                      float *id_ref_out, float *iq_ref_out);
/** SVPWM 前最终 Ud/Uq 已知后调用，完成 VASI 半周 ψ 积分 */
void ld_lq_ident_integrate(float id_fb, float iq_fb, float ud_out, float uq_out);
float ld_lq_ident_u_inj_d(void);
float ld_lq_ident_u_inj_q(void);
/** 1=INJ_LD / INJ_LQ 半周积分中 */
uint8_t ld_lq_ident_inject_active(void);
/** 0=开环注入段（PI 应暂停）；1=SETTLE 或旧闭环 VASI */
uint8_t ld_lq_ident_pi_active(void);
#if M1_LD_LQ_IDENT_OPEN_LOOP_ENABLE
float ld_lq_ident_u_bias_d(void);
float ld_lq_ident_u_bias_q(void);
#endif
uint8_t ld_lq_ident_is_done(void);
uint8_t ld_lq_ident_is_ok(void);
uint8_t ld_lq_ident_open_seq_phase(void);
void ld_lq_ident_get_result(ld_lq_ident_result_t *out);
void ld_lq_ident_sync_dbg(void);
#if M1_LD_LQ_MULTI_ANGLE_ENABLE
void ld_lq_ident_set_angle_leg(uint8_t leg, float theta_el_rad);
void ld_lq_ident_commit_leg(uint8_t leg);
void ld_lq_ident_get_result_leg(uint8_t leg, ld_lq_ident_result_t *out);
uint8_t ld_lq_ident_angle_leg(void);
#else
static inline void ld_lq_ident_set_angle_leg(uint8_t leg, float theta_el_rad)
{
    (void)leg;
    (void)theta_el_rad;
}
static inline void ld_lq_ident_commit_leg(uint8_t leg) { (void)leg; }
static inline void ld_lq_ident_get_result_leg(uint8_t leg, ld_lq_ident_result_t *out)
{
    if (out != 0) {
        *out = (ld_lq_ident_result_t){0};
    }
    (void)leg;
}
static inline uint8_t ld_lq_ident_angle_leg(void) { return 0u; }
#endif

#else

static inline void ld_lq_ident_init(void) {}
static inline void ld_lq_ident_arm(float rs_ohm)
{
    (void)rs_ohm;
}
static inline void ld_lq_ident_first_grid_bias(float *id0, float *iq0)
{
    if (id0 != 0) {
        *id0 = 0.0f;
    }
    if (iq0 != 0) {
        *iq0 = 0.0f;
    }
}
static inline void ld_lq_ident_tick(float id_fb, float iq_fb, float ud_pi, float uq_pi,
                                    float *id_ref_out, float *iq_ref_out)
{
    (void)id_fb;
    (void)iq_fb;
    (void)ud_pi;
    (void)uq_pi;
    if (id_ref_out != 0) {
        *id_ref_out = 0.0f;
    }
    if (iq_ref_out != 0) {
        *iq_ref_out = 0.0f;
    }
}
static inline void ld_lq_ident_integrate(float id_fb, float iq_fb, float ud_out, float uq_out)
{
    (void)id_fb;
    (void)iq_fb;
    (void)ud_out;
    (void)uq_out;
}
static inline float ld_lq_ident_u_inj_d(void) { return 0.0f; }
static inline float ld_lq_ident_u_inj_q(void) { return 0.0f; }
static inline uint8_t ld_lq_ident_inject_active(void) { return 0u; }
static inline uint8_t ld_lq_ident_pi_active(void) { return 0u; }
static inline float ld_lq_ident_u_bias_d(void) { return 0.0f; }
static inline float ld_lq_ident_u_bias_q(void) { return 0.0f; }
static inline uint8_t ld_lq_ident_is_done(void) { return 1u; }
static inline uint8_t ld_lq_ident_is_ok(void) { return 0u; }
static inline uint8_t ld_lq_ident_open_seq_phase(void) { return 9u; }
static inline void ld_lq_ident_get_result(ld_lq_ident_result_t *out)
{
    if (out != 0) {
        *out = (ld_lq_ident_result_t){0};
    }
}
static inline void ld_lq_ident_sync_dbg(void) {}

#endif /* M1_LD_LQ_IDENT_ENABLE */

#ifdef __cplusplus
}
#endif

#endif /* LD_LQ_IDENT_H */
