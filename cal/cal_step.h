/**
 * @file cal_step.h
 * @date 2026-10-06
 * @brief 出厂整定可插拔步骤接口（shape only；runner / steps 在 P1 实现）。
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
 */

#ifndef CAL_STEP_H
#define CAL_STEP_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 步骤 ID：只追加，不改已有数值 */
typedef enum {
    /** 电流采样零偏。 */
    CAL_STEP_ADC_ZERO     = 0x01u,
    /** 编码器电角零偏。 */
    CAL_STEP_ENCODER_ZERO = 0x02u,
    /** 定子电阻辨识。 */
    CAL_STEP_RS_IDENT     = 0x08u,
} cal_step_id_t;

/** tick 返回值 */
#define CAL_STEP_TICK_DONE    0
#define CAL_STEP_TICK_RUNNING 1

typedef struct cal_step_ops {
    /** cal_step_id_t。 */
    uint16_t id;
    /** 前置步骤 bitmask。 */
    uint16_t required_mask;
    /** 进入本步。0=成功。 */
    int (*begin)(void *ctx);
    /** 节拍。0=完成，1=进行中，<0=失败。 */
    int (*tick)(void *ctx);
    /** 中途放弃。 */
    void (*abort)(void *ctx);
    /** 收尾写结果。0=成功。 */
    int (*finish)(void *ctx);
} cal_step_ops_t;

#ifdef __cplusplus
}
#endif

#endif
