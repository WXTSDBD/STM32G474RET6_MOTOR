/**
 * @file bsp_axes.h
 * @brief 电机轴级 binding：每轴 ADC 采样 + 编码器 + PWM 定时器句柄聚合。
 *
 * 应用层通过 bsp_axis(BSP_AXIS_M1) 访问；HAL 指针由 bridge_cubemx 填入。
 * 函数说明见 bsp_axes.c。
 */

#ifndef BSP_AXES_H
#define BSP_AXES_H

#include <stdbool.h>
#include <stdint.h>

#include "adc_sample.h"
#include "encoder.h"
#include "tim.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    BSP_AXIS_M1 = 0,
    BSP_AXIS_M2 = 1,
    BSP_AXIS_COUNT
} bsp_axis_id_t;

/** 单根电机轴：enabled 为 false 时跳过 adc init / 标定 */
typedef struct {
    bool enabled;
    adc_sample_t adc;
    adc_sample_config_t adc_cfg;
    encoder_t *enc;
    TIM_HandleTypeDef *pwm_tim;
    void *motor_ctx;
} bsp_axis_t;

void bsp_init(void);

bsp_axis_t *bsp_axis(bsp_axis_id_t id);

bool bsp_axis_adc_calibrate_zero(bsp_axis_id_t id,
                                 uint16_t discard,
                                 uint16_t samples,
                                 uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif
