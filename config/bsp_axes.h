/**
 * @file bsp_axes.h
 * @date 2026-10-06
 * @brief 电机轴绑定：ADC、编码器、PWM 口。

 *
 * init 上电调用。电流环用 bsp_axis(M1)。
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
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
    /** 第一轴，本板电流环。 */
    BSP_AXIS_M1 = 0,
    /** 第二轴占位。 */
    BSP_AXIS_M2 = 1,
    /** 轴数量。 */
    BSP_AXIS_COUNT
} bsp_axis_id_t;

typedef struct {
    /** 1=本轴参与初始化。 */
    bool enabled;
    /** 电流采样状态。 */
    adc_sample_t adc;
    /** 电流采样配置，含 HAL 指针。 */
    adc_sample_config_t adc_cfg;
    /** 本轴编码器。可为 NULL。 */
    encoder_t *enc;
    /** ADC 注入完成口。 */
    adc_foc_port_t *adc_foc;
    /** PWM 口。 */
    pwm_port_t *pwm;
    /** 控制上下文。M1 指向 motor_current 内的实例。 */
    void *motor_ctx;
} bsp_axis_t;

/**
 * 诊断用分流 ADC 快照（3 路：ADC1/3/5）。
 * 仅 main 诊断回调写入；M1 三相 raw 走 `bsp_axis()->adc` / dbg，不再镜像到本缓冲。
 */
extern int16_t adc_read[3];

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
