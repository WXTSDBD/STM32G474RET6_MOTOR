/**
 * @file bsp_axes.h
 * @brief 电机轴级 binding：ADC + 编码器 + Port 聚合。
 *
 * HAL 语义配置由 bsp_axes 填写；句柄与 Port 实例由 bridge_cubemx + board 绑定。
 */

#ifndef BSP_AXES_H
#define BSP_AXES_H

#include <stdbool.h>
#include <stdint.h>

#include "adc_foc_port.h"
#include "adc_sample.h"
#include "encoder.h"
#include "pwm_port.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    BSP_AXIS_M1 = 0,
    BSP_AXIS_M2 = 1,
    BSP_AXIS_COUNT
} bsp_axis_id_t;

typedef struct {
    bool enabled;
    adc_sample_t adc;
    adc_sample_config_t adc_cfg;
    encoder_t *enc;
    adc_foc_port_t *adc_foc;
    pwm_port_t *pwm;
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
