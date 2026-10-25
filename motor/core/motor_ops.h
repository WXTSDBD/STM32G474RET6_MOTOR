/**
 * @file motor_ops.h
 * @date 2026-10-25
 * @brief 五张函数指针表（"宏转表" 阶段 0 · 冻结接口）。
 *
 * 对应设计：docs/方案_宏转实验表_设计_2026-10-25.md §4/§5.3。
 * 目标：函数**总是编译**，"选谁"由表指针决定，不由 `#if M1_*` 决定。
 *
 * 现网 ±2000：config/routes 在绑定时装上 angle，段边界再换指针。
 * 20 kHz 只解引用，不在热路径里选择角度源。
 */
#ifndef MOTOR_CORE_MOTOR_OPS_H
#define MOTOR_CORE_MOTOR_OPS_H

#include <stdint.h>

#include "../motor_context.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ===== 1) 角度源：有感 / 编码器 PLL / SMO / HFI / 复合 / I-F ===== */
typedef struct {
    /** 每 20 kHz 拍推进观测器（有感可为空）。 */
    void (*update)(motor_context_t *ctx, float dt_s);
    /** 取本拍 Park 用的电角度（rad）。 */
    float (*get_theta)(motor_context_t *ctx);
    /** 取速度反馈（机端 rpm）。 */
    float (*get_omega_rpm)(motor_context_t *ctx);
    /**
     * Park 后解调。有感源填 NULL，20 kHz 看到 NULL 就不跑无感解调。
     */
    void (*post_park)(motor_context_t *ctx, float id, float iq,
                      float i_alpha, float i_beta);
    /** 源切换时的软交接（可为空）。 */
    void (*handoff)(motor_context_t *ctx, uint8_t from_ops_id);
} angle_ops_t;

/* ===== 2) 外环：torque / speed / pos / mit ===== */
typedef struct {
    /** 每 2 kHz 拍推进一次；返回 iq_ref（A）。 */
    float (*tick)(motor_context_t *ctx, float theta_fb_rad, float omega_rpm);
    /** 进入该外环模式时复位状态（可为空）。 */
    void (*enter)(motor_context_t *ctx);
} outer_ops_t;

/* ===== 3) 前馈链：解耦 / 摩擦 / ESO / 死区 ===== */
typedef struct {
    /** 输入输出均为 dq 电压增量；可为空（表示该级不参与）。 */
    void (*apply)(motor_context_t *ctx, float *ud_out, float *uq_out, float id, float iq);
} ff_ops_t;

/* ===== 4) 电流给定：zero_d / MTPA / 弱磁（预留） ===== */
typedef struct {
    /** 由 iq_ref 产出 id_ref（及可选修正后的 iq_ref）。 */
    void (*ref)(motor_context_t *ctx, float iq_cmd, float *id_ref, float *iq_ref);
} iref_ops_t;

/* ===== 5) 遥测：12 通道布局表 ===== */
typedef struct {
    uint8_t layout_id;
    const char *name;
} telem_ops_t;

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_CORE_MOTOR_OPS_H */
