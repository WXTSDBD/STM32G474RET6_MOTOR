/**
 * @file exp_mark.h
 * @date 2026-10-07
 * @brief open_seq / 签收打点号符号化。
 *
 * 包 8d 改号（行为包，须台架短回归）：
 *   - OUTER_S3_PROBE_DONE：243 → **209**（解除与 IF_OBS_HANDED 双占）
 *   - S3 探针步：硬编码 240/241/242 → **206/207/208**（避开 IF 带）
 *   - CRUISE_STEP_BASE：250 → **233**（233..238；远离 252/255，避开 200 段 reversal/deadband）
 * 金样脚本认数值不认宏名；HFI stage 整段写入仍另案。
 */

#ifndef DEBUG_EXP_MARK_H
#define DEBUG_EXP_MARK_H

#ifdef __cplusplus
extern "C" {
#endif

/* ---- IF / 观测交接（motor_current） ---- */
#define EXP_MARK_IF_BRINGUP            240u
#define EXP_MARK_IF_OBS_ANGLE_ONLY     242u
#define EXP_MARK_IF_OBS_HANDED         243u
#define EXP_MARK_IF_OBS_BLEND_WEAK     244u
#define EXP_MARK_IF_OBS_BLEND_SPEED    245u

/* ---- 巡航 / 外环（motor_outer_loop） ---- */
#define EXP_MARK_CRUISE_SOFT_BRAKE     246u
#define EXP_MARK_CRUISE_AUTH_RAMP      247u
#define EXP_MARK_CRUISE_PI             248u
#define EXP_MARK_CRUISE_STAGE3_WAIT    249u
/** 巡航硬阶跃表：实际写 BASE+idx，idx∈[0,5] → 233..238。 */
#define EXP_MARK_CRUISE_STEP_BASE      233u
/** S3 探针：HI / LO / 回基 / 完。 */
#define EXP_MARK_S3_PROBE_HI           206u
#define EXP_MARK_S3_PROBE_LO           207u
#define EXP_MARK_S3_PROBE_BASE         208u
#define EXP_MARK_OUTER_S3_PROBE_DONE   209u /* 原 243，避开 IF_OBS_HANDED */
#define EXP_MARK_SIGN_DONE             255u
#define EXP_MARK_SIGN_GUARD            252u

/* ---- 速度 ident / 其它共用 ---- */
#define EXP_MARK_SPEED_IDENT_RUN       230u
#define EXP_MARK_SPEED_IDENT_BLEND     231u
#define EXP_MARK_SPEED_IDENT_DONE      232u
#define EXP_MARK_SPEED_STEP_TABLE_END  239u

#if (EXP_MARK_CRUISE_STEP_BASE < (EXP_MARK_SPEED_IDENT_DONE + 1u))
#error "EXP_MARK_CRUISE_STEP_BASE must sit above SPEED_IDENT_DONE (232)"
#endif
#if ((EXP_MARK_CRUISE_STEP_BASE + 5u) >= EXP_MARK_SPEED_STEP_TABLE_END)
#error "CRUISE_STEP BASE+idx must end before SPEED_STEP_TABLE_END (239)"
#endif
#if (EXP_MARK_IF_OBS_HANDED == EXP_MARK_OUTER_S3_PROBE_DONE)
#error "IF_OBS_HANDED and OUTER_S3_PROBE_DONE must differ"
#endif
#if (EXP_MARK_S3_PROBE_HI == EXP_MARK_OUTER_S3_PROBE_DONE) || \
    (EXP_MARK_S3_PROBE_LO == EXP_MARK_OUTER_S3_PROBE_DONE) || \
    (EXP_MARK_S3_PROBE_BASE == EXP_MARK_OUTER_S3_PROBE_DONE)
#error "S3 probe step marks must differ from OUTER_S3_PROBE_DONE"
#endif
#if (EXP_MARK_CRUISE_STEP_BASE + 5u) >= EXP_MARK_SIGN_GUARD
#error "cruise step range must stay below SIGN_GUARD (252)"
#endif

#ifdef __cplusplus
}
#endif

#endif /* DEBUG_EXP_MARK_H */
