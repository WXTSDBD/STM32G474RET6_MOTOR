/**
 * @file cal_step.h
 * @brief 出厂整定可插拔步骤接口（shape only；runner / steps 在 P1 实现）。
 */

#ifndef CAL_STEP_H
#define CAL_STEP_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 步骤 ID：只追加，不改已有数值 */
typedef enum {
    CAL_STEP_ADC_ZERO     = 0x01u,
    CAL_STEP_ENCODER_ZERO = 0x02u,
    CAL_STEP_RS_IDENT     = 0x08u,
} cal_step_id_t;

/** tick 返回值 */
#define CAL_STEP_TICK_DONE    0
#define CAL_STEP_TICK_RUNNING 1

typedef struct cal_step_ops {
    uint16_t id;
    uint16_t required_mask; /**< 前置步骤 bitmask */
    int (*begin)(void *ctx);
    int (*tick)(void *ctx);   /**< 0=完成, 1=进行中, <0=失败 */
    void (*abort)(void *ctx);
    int (*finish)(void *ctx); /**< 0=成功 */
} cal_step_ops_t;

#ifdef __cplusplus
}
#endif

#endif
