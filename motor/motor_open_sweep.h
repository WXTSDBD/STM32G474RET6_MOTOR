/**
 * @file motor_open_sweep.h
 * @brief 开环 Uq/Ud 可插拔：V0/V1/V2 扫参、AB 死区对比、固定 Uq 定时、Id cal 前阶梯。
 */

#ifndef MOTOR_OPEN_SWEEP_H
#define MOTOR_OPEN_SWEEP_H

#include "motor_context.h"

#ifdef __cplusplus
extern "C" {
#endif

void motor_open_sweep_init(motor_context_t *ctx);

/** 上电 OBSERVE/OPEN 模式：2/2.5/3 V 或 AB 六段扫参。 */
void motor_open_sweep_arm_v012(motor_context_t *ctx);

/** 固定 Uq（V）运行 duration_s 秒；调用后需 ctx->mode=OPEN/OBSERVE。 */
void motor_open_sweep_begin_fixed(motor_context_t *ctx, float uq_v, float duration_s);

#if M1_OPEN_UQ_PRE_ID_CAL_ENABLE
/** 实验 A：ALIGN(68) → Uq 0/0.2/0.5/1/2/4 V @30°；dwell 0.3 s；phase 70..77。 */
void motor_open_sweep_begin_pre_id_uq_ladder(motor_context_t *ctx);
#endif

#if M1_OPEN_UD_PRE_ID_CAL_ENABLE
/** 实验 B：ALIGN(68) → Ud OFF 70..75 → LUT ON 80..85（AB=1）；dwell 0.3 s。 */
void motor_open_sweep_begin_pre_id_ud_ladder(motor_context_t *ctx);
#endif

#if M1_OPEN_UQ_PRE_ID_CAL_ENABLE || M1_OPEN_UD_PRE_ID_CAL_ENABLE
/** ALIGN 或 FIX_THETA 阶梯：Park/SVPWM 用 M1_OPEN_PRE_ID_LADDER_THETA_EL_RAD。 */
uint8_t motor_open_sweep_use_fix_theta(void);

/** ALIGN 段：开环 Ud 吸转子，Uq=0。 */
uint8_t motor_open_sweep_in_pre_id_align(void);

/** 实验 B 阶梯段：开环 Ud，Uq=0。 */
uint8_t motor_open_sweep_in_pre_id_ud_ladder(void);
#endif

/** OPEN/OBSERVE 模式下每 JEOC 调用，更新 ctx->uq_open / ctx->ud_open。 */
void motor_open_sweep_tick(motor_context_t *ctx);

uint8_t motor_open_sweep_active(void);
uint8_t motor_open_sweep_done(void);

#ifdef __cplusplus
}
#endif

#endif
