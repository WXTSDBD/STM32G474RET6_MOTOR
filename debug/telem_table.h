/**
 * @file telem_table.h
 * @date 2026-10-07
 * @brief 遥测 12ch 表驱动（T-2）：layout 索引 + getter 直读状态。
 *
 * 打包侧用；ISR 热路径不调用本头 getter。帧头 layout_id 默认关。
 * 节拍限制见 app_uart_dma_debug.h。
 */

#ifndef DEBUG_TELEM_TABLE_H
#define DEBUG_TELEM_TABLE_H

#include <stdint.h>

#include "motor_context.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 每 layout 固定 12 通道（与 M1_TELEM_BRINGUP_K 对齐）。 */
#define TELEM_TABLE_CH_N           12u

/**
 * 帧是否在 12ch 前多写 1 个 float 的 layout_id。
 * 默认关：占带宽、VOFA 多一路，且运行时切 layout 未用上。
 * 真要运行时切表时再改为 1，并同步 VOFA/脚本。
 */
#ifndef TELEM_FRAME_LAYOUT_ID
#define TELEM_FRAME_LAYOUT_ID      0u
#endif

#define TELEM_LAYOUT_FOC_DQ        0u /* ia ib ic id iq θ ud uq … 旧统一前半 */
#define TELEM_LAYOUT_SIGNOFF       1u /* 有感/MIT 签收 12ch */
#define TELEM_LAYOUT_SPEED         2u /* 速度环：ω_pll ω_ref iq_ref mark */
#define TELEM_LAYOUT_IDENT_DUTY    3u /* 辨识 duty / Vd Vq */
#define TELEM_LAYOUT_LEGACY        255u /* 走旧 #elif 手填 */

typedef struct {
    const char *name;
    float (*get)(const motor_context_t *ctx);
} telem_ch_t;

void telem_table_init(void);
void telem_set_layout(uint8_t layout_id);
uint8_t telem_get_layout(void);

/**
 * @brief 按当前 layout 填 12 通道。
 * @return 1=已填（表驱动）；0=当前是 LEGACY，调用方走旧分支。
 */
uint8_t telem_table_fill(float vals[TELEM_TABLE_CH_N],
                         const motor_context_t *ctx);

#ifdef __cplusplus
}
#endif

#endif /* DEBUG_TELEM_TABLE_H */
