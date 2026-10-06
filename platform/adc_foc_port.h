/**
 * @file adc_foc_port.h
 * @date 2026-10-06
 * @brief 电流环 ADC 注入完成口。不含 HAL。

 *
 * on_jeoc 只允许从 ADC 注入转换完成中断调用。
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
 */

#ifndef ADC_FOC_PORT_H
#define ADC_FOC_PORT_H

#include "adc_sample.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct adc_foc_port adc_foc_port_t;

typedef struct adc_foc_port_ops {
    void (*on_jeoc)(adc_foc_port_t *port, adc_sample_t *sample, void *hadc);
} adc_foc_port_ops_t;

struct adc_foc_port {
    const adc_foc_port_ops_t *ops;
    void *hw;
};

extern const adc_foc_port_ops_t adc_foc_port_ops_stm32g4_reg;

void adc_foc_port_on_jeoc(adc_foc_port_t *port, adc_sample_t *sample, void *hadc);

#ifdef __cplusplus
}
#endif

#endif
