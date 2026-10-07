/**
 * @file exp_def.h
 * @date 2026-10-07
 * @brief 实验框架类型：段表 / 实验定义 / 脚本接口（E0）。
 *
 * 执行器只在 2 kHz 外环调用。加实验 = 加表 + 注册表一行，不碰热路径。
 * 判据留 PC；固件只发激励、打点、留数。
 */

#ifndef MOTOR_EXPERIMENT_EXP_DEF_H
#define MOTOR_EXPERIMENT_EXP_DEF_H

#include <stdint.h>

#include "motor_context.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 实验类别。只有验收/对照/整定进注册表；探索继续用宏。 */
#define EXP_KIND_SCHEDULE   0u
#define EXP_KIND_SCRIPT     1u
#define EXP_KIND_CALIB      2u

/** 默认动作 id；0xFF=本段用 ops 或由 SCRIPT 接管。 */
#define EXP_ACT_NONE        0xFFu
#define EXP_ACT_HOLD        0u
#define EXP_ACT_THETA_REF   1u
#define EXP_ACT_OMEGA_REF   2u
#define EXP_ACT_IQ_FF       3u

/** 已注册实验 id。 */
#define EXP_ID_NONE         0u
#define EXP_ID_SIGNOFF      1u

typedef struct exp_seg_s exp_seg_t;

typedef struct {
    void (*enter)(motor_context_t *ctx, const exp_seg_t *seg);
    void (*update)(motor_context_t *ctx, const exp_seg_t *seg, float t_s);
} exp_action_ops_t;

struct exp_seg_s {
    /** 段时长，单位 ms。0=由 SCRIPT / ops 自己决定何时结束。 */
    uint16_t dur_ms;
    /** 默认动作；EXP_ACT_NONE 时用 ops。 */
    uint8_t act;
    const exp_action_ops_t *ops;
    float p0;
    float p1;
    float p2;
    /** 对照配置下标；0=A。E0 不用。 */
    uint8_t cfg_idx;
    /** 打点号基值 → 遥测 mark（与现网 open_seq 段基对齐）。 */
    uint8_t mark;
    /** 本段守卫掩码；E0 保留字段，现网守卫仍走 outer 原逻辑。 */
    uint8_t guard_mask;
};

typedef struct {
    uint8_t ff_en;
    uint8_t eso_en;
    uint8_t deadband_en;
    uint8_t theta_src;
    float pi_kp;
    float pi_ki;
} exp_cfg_t;

typedef struct {
    void (*init)(motor_context_t *ctx, const exp_cfg_t *cfg);
    void (*tick)(motor_context_t *ctx, float theta_fb_rad, float omega_rpm);
    uint8_t (*done)(const motor_context_t *ctx);
} exp_script_ops_t;

typedef struct {
    uint8_t id;
    const char *name;
    uint8_t kind;
    const exp_seg_t *seg;
    uint16_t seg_n;
    const exp_cfg_t *cfg;
    uint8_t cfg_n;
    uint8_t telem_layout;
    const exp_script_ops_t *script;
} exp_def_t;

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_EXPERIMENT_EXP_DEF_H */
