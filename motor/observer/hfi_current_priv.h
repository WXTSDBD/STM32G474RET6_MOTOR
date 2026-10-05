/* P5: moved HFI helpers. Tick call order unchanged. */
#ifndef MOTOR_OBSERVER_HFI_CURRENT_PRIV_H
#define MOTOR_OBSERVER_HFI_CURRENT_PRIV_H

#include <stdint.h>
#include "observer/obs_cfg.h"
#include "motor_context.h"
#include "observer/emf_pll.h"

#ifndef HFI_SMO_W_MA_N
#define HFI_SMO_W_MA_N 400u
#endif

#if M1_HFI_SMO_HAND_ENABLE && M1_EMF_PLL_ENABLE
#ifndef M1_HFI_HAND_ID_OVERLAP_ENABLE
#define M1_HFI_HAND_ID_OVERLAP_ENABLE   0
#endif
#ifndef M1_HFI_HAND_OPEN_ID_ENABLE
#define M1_HFI_HAND_OPEN_ID_ENABLE      0 /* 1：角交后在残 Vh 下开 Id */
#endif
#ifndef M1_HFI_SMO_SUB_VH_ENABLE
#define M1_HFI_SMO_SUB_VH_ENABLE        0 /* 1：SMO 用 u−u_hfi，HFI 段不停观测 */
#endif
#ifndef M1_HFI_ROTATE_PI_ENABLE
#define M1_HFI_ROTATE_PI_ENABLE         0 /* 1：Park 切 SMO 时旋 Id/Iq PI */
#endif
#ifndef M1_HFI_ID_ON_FROM_RUN_ENABLE
#define M1_HFI_ID_ON_FROM_RUN_ENABLE    0
#endif
#ifndef M1_HFI_ID_PI_OFF_ENABLE
#define M1_HFI_ID_PI_OFF_ENABLE         0
#endif
#ifndef M1_HFI_HAND_IQ_HOLD_ON_IDUP
#define M1_HFI_HAND_IQ_HOLD_ON_IDUP     0 /* 诊断用：。iq_ref；产品路径用 W_HOLD */
#endif
#ifndef M1_HFI_HAND_W_HOLD_ON_IDUP
#define M1_HFI_HAND_W_HOLD_ON_IDUP      0 /* 1：开 Id 窗速度环吃 VH0 。ω，SMO 软释。*/
#endif
#ifndef M1_HFI_HAND_W_REL_N
#define M1_HFI_HAND_W_REL_N            4000u /* W_HOLD 释放。.2 s @ 20 kHz */
#endif
#ifndef M1_HFI_HAND_W_SLEW_ENABLE
#define M1_HFI_HAND_W_SLEW_ENABLE       0 /* 1: slew-limit omega_fb */
#endif
#ifndef M1_HFI_HAND_W_SLEW_RPM_S
#define M1_HFI_HAND_W_SLEW_RPM_S        (200.0f)
#endif
#ifndef M1_HFI_HAND_W_SLEW_IDUP_ONLY
#define M1_HFI_HAND_W_SLEW_IDUP_ONLY    0 /* 1: only IDUP(+SMO_N); 0: ANG..SMO (63) */
#endif
#ifndef M1_HFI_HAND_W_SLEW_SMO_N
#define M1_HFI_HAND_W_SLEW_SMO_N       4000u /* IDUP_ONLY: continue slew 0.2 s into SMO */
#endif
#ifndef M1_HFI_HAND_KILL_VH_ENABLE
#define M1_HFI_HAND_KILL_VH_ENABLE      0
#endif
#ifndef M1_HFI_HAND_VH0_SOFT_ENABLE
#define M1_HFI_HAND_VH0_SOFT_ENABLE     0 /* 1：smoothstep 。Vh */
#endif
#ifndef M1_HFI_HAND_STOP_AFTER
#define M1_HFI_HAND_STOP_AFTER          0
#endif
#ifndef M1_HFI_HAND_VH_FLOOR
#define M1_HFI_HAND_VH_FLOOR            (0.25f) /* FADE/ANG 残注。scale */
#endif
#ifndef M1_HFI_HAND_VH_END
#define M1_HFI_HAND_VH_END              (0.0f) /* KILL 终点 scale。0=微地。*/
#endif
#ifndef M1_HFI_HAND_ID_WEAK
#define M1_HFI_HAND_ID_WEAK             (0.12f)
#endif
#ifndef M1_HFI_HAND_HOLD_N
#define M1_HFI_HAND_HOLD_N             10000u /* 0.5 s：ang=1 后冻。*/
#endif
#ifndef M1_HFI_HAND_VH0_N
#define M1_HFI_HAND_VH0_N              20000u /* 地板→END 时长 */
#endif
#ifndef M1_HFI_HAND_IDUP_N
#define M1_HFI_HAND_IDUP_N             20000u /* 1.0 s：只。Id */
#endif
#ifndef M1_HFI_HAND_FADE_N
#define M1_HFI_HAND_FADE_N             4000u /* 1→地板时。@ 20 kHz */
#endif
/* SMO→HFI 反向。9 对称串行。先抬微地板→残地板，再交角、满注入、交速。*/
#ifndef M1_HFI_HAND_REV_ENABLE
#define M1_HFI_HAND_REV_ENABLE          0
#endif
#ifndef M1_HFI_HAND_REV_WAKE_ENABLE
#define M1_HFI_HAND_REV_WAKE_ENABLE     0 /* 1：RVH 。hold 。θ̂，RQUAL 过门。RANG */
#endif
#ifndef M1_HFI_HAND_REV_VH_WAKE
#define M1_HFI_HAND_REV_VH_WAKE         M1_HFI_HAND_VH_FLOOR /* 唤醒目标 scale。8=1.0 */
#endif
#ifndef M1_HFI_HAND_REV_RVH_HOLD_ENABLE
#define M1_HFI_HAND_REV_RVH_HOLD_ENABLE 0 /* 1：RVH 。θ̂=SMO（hold+coast），RQUAL 再放 PLL */
#endif
#ifndef M1_HFI_HAND_REV_RESEED_N
#define M1_HFI_HAND_REV_RESEED_N       2000u /* RVH hold 时每 0.1 s 。seed */
#endif
#ifndef M1_HFI_HAND_RQUAL_X_MAX_ENABLE
#define M1_HFI_HAND_RQUAL_X_MAX_ENABLE  1 /* 0：去。x 上界。9。8 上界误杀健康解调。*/
#endif
#ifndef M1_HFI_HAND_RQUAL_X_MAX
#define M1_HFI_HAND_RQUAL_X_MAX         (0.45f)
#endif
#ifndef M1_HFI_HAND_REV_OBS_ENABLE
#define M1_HFI_HAND_REV_OBS_ENABLE      0 /* 1：RVH→ROBS 旁路观察，不交角/速（70） */
#endif
#ifndef M1_HFI_HAND_REV_VH_MIRROR_ENABLE
#define M1_HFI_HAND_REV_VH_MIRROR_ENABLE 0 /* 1：Vh 按前向 VH0+FADE 反演抬（71） */
#endif
#ifndef M1_HFI_HAND_DECEL_BRAKE_ENABLE
#define M1_HFI_HAND_DECEL_BRAKE_ENABLE  0 /* 1：SMO 减速制动向 Iq 地板（75） */
#endif
#ifndef M1_HFI_HAND_DECEL_BRAKE_IQ_A
#define M1_HFI_HAND_DECEL_BRAKE_IQ_A    (1.5f) /* 制动 |Iq|_min；按 -sign(ω) 抬，不保巡航正号 */
#endif
#ifndef M1_HFI_HAND_DECEL_BRAKE_END_RPM
#define M1_HFI_HAND_DECEL_BRAKE_END_RPM (920.0f) /* SMO ω 落到此再交回速度环 */
#endif
#ifndef M1_HFI_HAND_DECEL_BRAKE_ARM_RPM
#define M1_HFI_HAND_DECEL_BRAKE_ARM_RPM (1400.0f) /* SMO ω 先爬过再允许地板 */
#endif
#ifndef M1_HFI_HAND_DECEL_BRAKE_DROP_RPM
#define M1_HFI_HAND_DECEL_BRAKE_DROP_RPM (80.0f) /* ω* 比 SMO ω 低这么多才算减速意图 */
#endif
#ifndef M1_HFI_HAND_SPD_X_KILL_HI
#define M1_HFI_HAND_SPD_X_KILL_HI       (0.62f) /* SPD 混速 x 杀门；70=0.70 避 1530 */
#endif
#ifndef M1_HFI_HAND_REV_RPM
#define M1_HFI_HAND_REV_RPM             (1400.0f) /* 减速过线触发回 HFI */
#endif
#ifndef M1_HFI_HAND_REV_ARM_RPM
#define M1_HFI_HAND_REV_ARM_RPM         (1450.0f) /* 须先爬过再允许反。*/
#endif
#ifndef M1_HFI_HAND_RVH_N
#define M1_HFI_HAND_RVH_N              20000u /* 1.0 s：END→FLOOR */
#endif
#ifndef M1_HFI_HAND_RQUAL_N
#define M1_HFI_HAND_RQUAL_N            8000u /* 0.4 s：θ。锁门，对。QUAL */
#endif
#ifndef M1_HFI_HAND_RQUAL_TIMEOUT_N
#define M1_HFI_HAND_RQUAL_TIMEOUT_N    60000u /* 3.0 s 锁不上则退。SMO */
#endif
#ifndef M1_HFI_HAND_RANG_N
#define M1_HFI_HAND_RANG_N             20000u /* 1.0 s：ang 1。 */
#endif
#ifndef M1_HFI_HAND_RFADE_N
#define M1_HFI_HAND_RFADE_N            16000u /* FLOOR。 */
#endif
#ifndef M1_HFI_HAND_RSPD_N
#define M1_HFI_HAND_RSPD_N             40000u /* 2.0 s：alpha 1。 */
#endif
#if M1_HFI_HAND_ID_OVERLAP_ENABLE || M1_HFI_HAND_OPEN_ID_ENABLE
#define M1_HFI_HAND_ID_WITH_VH          1
#else
#define M1_HFI_HAND_ID_WITH_VH          0
#endif
#define HFI_HAND_HFI   0u
#define HFI_HAND_QUAL  1u
#define HFI_HAND_SPD   2u
#define HFI_HAND_CONF  3u
#define HFI_HAND_FADE  4u
#define HFI_HAND_ANG   5u
#define HFI_HAND_HOLD  6u /* OVERLAP：满 Park + 。Vh + 。Id */
#define HFI_HAND_VH0   7u /* 地板→END；OVERLAP 。Id=WEAK，KILL 。Id 旁路 */
#define HFI_HAND_IDUP  8u /* OVERLAP：Vh=END，Id WEAK。 */
#define HFI_HAND_SMO   9u
#define HFI_HAND_RVH   10u /* 反向：vh END→FLOOR，Park 。SMO */
#define HFI_HAND_RANG  11u /* 反向：ang 1。 */
#define HFI_HAND_RFADE 12u /* 反向：vh FLOOR。 */
#define HFI_HAND_RSPD  13u /* 反向：alpha 1。 。HFI */
#define HFI_HAND_RQUAL 14u /* 反向：θ。重锁门（WAKE。*/
#define HFI_HAND_ROBS  15u /* reverse observe bypass */
#define HFI_HAND_QUAL_N  8000u  /* 0.4 s @ 20 kHz */
#define HFI_HAND_BLEND_N 40000u /* 2.0 s，只交速度 */
#define HFI_HAND_CONF_N  4000u  /* 0.2 s，速度交完后再看一。*/
#define HFI_HAND_ANG_N   20000u /* 1.0 s */
#define HFI_HAND_ANG_OK  0.436332f /* 25° */
#define HFI_HAND_ANG_ABORT 0.523599f /* 30° */
#define HFI_HAND_ANG_KILL  0.785398f /* 45° */
#define HFI_HAND_BAD_N   400u    /* 20 ms */
#endif

/* --- hfi_speed_filt.c --- */
void hfi_notch_coeff(float f_hz, float q, float fs, float *b0, float *b1, float *b2, float *a1, float *a2);
void hfi_notch_prime(float x, float *x1, float *x2, float *y1, float *y2);
float hfi_notch_run(float x, float b0, float b1, float b2, float a1, float a2, float *x1, float *x2, float *y1, float *y2);
float hfi_spd_notch_rpm(float rpm, float cmd);
void hfi_n24_coeff(float f_hz, float q, float fs, float *b0, float *b1, float *b2, float *a1, float *a2);
void hfi_n24_prime(float x, float *x1, float *x2, float *y1, float *y2);
float hfi_n24_run(float x, float b0, float b1, float b2, float a1, float a2, float *x1, float *x2, float *y1, float *y2);
float hfi_spd_notch24_rpm(float rpm, float cmd);
void hfi_lead_prime(float x);
void hfi_lead_coeff(float f_hz);
float hfi_spd_lead_rpm(float rpm, float cmd);
float hfi_ph_atan(float x);
void hfi_ph_pair(float rpm, float p, float phi, float *c, float *s, float *h, float *hadv);
float hfi_spd_phase_rpm(float rpm, float cmd);
void hfi_rip_cap(float *c, float *s);
void hfi_rip_decay(void);
float hfi_ripple_iq(float iq, float w_ref);
uint8_t hfi_spd_notch_on(void);
float hfi_rip_amp(void);
float hfi_spd_lead_rep(void);
float hfi_spd_ph_rep(void);

/* --- hfi_spd_obs.c --- */
void hfi_vesc_win_obs_update(float w_rpm);
void hfi_smo_w_ma_reset(void);
float hfi_smo_w_ma_step(float rpm);
void hfi_spd_shadow_step(float iq_hold, float w_fb_rpm);
uint8_t hfi_vesc_win_smo(void);
void hfi_vesc_win_smo_set(uint8_t v);
uint8_t hfi_vesc_ho_active(void);
uint32_t hfi_vesc_ho_n(void);
void hfi_vesc_ho_set(uint8_t active, uint32_t n);

/* --- hfi_hand.c --- */
float hfi_hand_smoothstep(float a);
void hfi_hand_rotate_current_pi(motor_context_t *ctx, float dth, float id, float iq);
float hfi_hand_w_el(void);
void hfi_smo_hand_idle(void);
void hfi_hand_abort(void);
uint8_t hfi_hand_watch(float ad, float dw, uint8_t spd_blend);
void hfi_hand_follow_smo(void);
void hfi_smo_hand_step(float w_hfi);
void hfi_hand_bind_pll(void *pll);
void hfi_hand_clear_ok(void);
void hfi_hand_note_smo(float theta, float w_rpm);
float hfi_hand_blend_speed_fb(float w_hfi);
void hfi_hand_rotate_if_due(motor_context_t *ctx, float theta_park,
                            float id, float iq);
void hfi_hand_iq_hold_apply(motor_context_t *ctx);
void hfi_hand_decel_brake_apply(motor_context_t *ctx);
uint8_t hfi_hand_ok(void);
uint8_t hfi_hand_state(void);
float hfi_hand_ang(void);
float hfi_hand_dth(void);

/* --- hfi_pub.c --- */
void hfi_pub_step(float theta_smo, float omega_el, float dth);

#endif /* MOTOR_OBSERVER_HFI_CURRENT_PRIV_H */

