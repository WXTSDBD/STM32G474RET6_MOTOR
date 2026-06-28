/**
 * @file encoder_cal.h
 * @brief 编码器电角零偏标定（锁转子 + 阻塞 SPI 读角）。
 *
 * JEOC 标定期间由 encoder_cal_jeoc_tick 固定 θ_ref 吸转子；
 * 正常 FOC 路径不得调用本模块。
 */

#ifndef ENCODER_CAL_H
#define ENCODER_CAL_H

#include <stdbool.h>
#include <stdint.h>

#include "bsp_axes.h"
#include "encoder.h"

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

#ifndef ENCODER_CAL_APPLY_PI_OFFSET
#define ENCODER_CAL_APPLY_PI_OFFSET 0
#endif

extern volatile uint8_t g_encoder_cal_active;

void encoder_cal_jeoc_tick(bsp_axis_t *axis);

float encoder_cal_wrap_2pi(float angle);
float encoder_cal_apply_pi_offset(float add_raw);

bool encoder_cal_run_lock(bsp_axis_t *axis,
                          encoder_t *enc,
                          uint8_t pole_pairs,
                          float uq_lock,
                          uint32_t settle_ms,
                          uint16_t sample_count,
                          float *add_out);

static inline bool encoder_cal_run_lock_default(bsp_axis_t *axis,
                                                encoder_t *enc,
                                                uint8_t pole_pairs,
                                                float *add_out)
{
    return encoder_cal_run_lock(axis, enc, pole_pairs,
                                ENCODER_CAL_UQ_LOCK,
                                ENCODER_CAL_SETTLE_MS,
                                ENCODER_CAL_SAMPLE_COUNT,
                                add_out);
}

#ifdef __cplusplus
}
#endif

#endif
