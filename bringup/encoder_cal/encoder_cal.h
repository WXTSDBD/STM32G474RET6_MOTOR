/**
 * @file encoder_cal.h
 * @brief 编码器电角零偏标定（方法 A：锁转子 + 阻塞 SPI 读角）。
 *
 * JEOC 标定期间由 encoder_cal_jeoc_tick 固定 θ_ref 吸转子；
 * 正常 FOC 路径不得调用本模块（见实施计划 §4.4）。
 */

#ifndef ENCODER_CAL_H
#define ENCODER_CAL_H

#include <stdbool.h>
#include <stdint.h>

#include "encoder.h"
#include "tim.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef ENCODER_CAL_UQ_LOCK
#define ENCODER_CAL_UQ_LOCK 2.5f
#endif

#ifndef ENCODER_CAL_SETTLE_MS
#define ENCODER_CAL_SETTLE_MS 400u
#endif

#ifndef ENCODER_CAL_SAMPLE_COUNT
#define ENCODER_CAL_SAMPLE_COUNT 16u
#endif

/** 1：锁转子 raw add 再减 π，消除 d/q 180° 歧义（与经验 add≈2.1 一致） */
#ifndef ENCODER_CAL_APPLY_PI_OFFSET
#define ENCODER_CAL_APPLY_PI_OFFSET 0
#endif

/** 1：JEOC 走 encoder_cal_jeoc_tick，不 kick、不 FOC */
extern volatile uint8_t g_encoder_cal_active;

/**
 * @brief JEOC 热路径：固定电角 SVPWM（θ_ref=0，Uq=uq_lock）。
 */
void encoder_cal_jeoc_tick(TIM_HandleTypeDef *htim);

/** wrap 到 [0, 2π)；供 π 修正 */
float encoder_cal_wrap_2pi(float angle);

/** add_raw − π（wrap）；ENCODER_CAL_APPLY_PI_OFFSET 为 0 时原样返回 */
float encoder_cal_apply_pi_offset(float add_raw);

/**
 * @brief 阻塞锁转子标定：等 settle → 阻塞读角 → 算 add → encoder_kick 预热。
 * @param add_out 输出 wrap 后的 theta_el_offset [rad]
 * @return false 参数或 chip_ctx 无效
 *
 * 顺序：active=1 等 settle → active=0 → 阻塞读 → add → kick（§4.4）
 */
bool encoder_cal_run_lock(encoder_t *enc,
                          TIM_HandleTypeDef *htim,
                          uint8_t pole_pairs,
                          float uq_lock,
                          uint32_t settle_ms,
                          uint16_t sample_count,
                          float *add_out);

/** 使用默认 ENCODER_CAL_* 宏的便捷封装 */
static inline bool encoder_cal_run_lock_default(encoder_t *enc,
                                                TIM_HandleTypeDef *htim,
                                                uint8_t pole_pairs,
                                                float *add_out)
{
    return encoder_cal_run_lock(enc, htim, pole_pairs,
                                ENCODER_CAL_UQ_LOCK,
                                ENCODER_CAL_SETTLE_MS,
                                ENCODER_CAL_SAMPLE_COUNT,
                                add_out);
}

#ifdef __cplusplus
}
#endif

#endif
