/**
 * @file adc_foc_port.h
 * @brief FOC 20 kHz ADC JEOC Port 契约（无 HAL）。P3 在 main ISR 接入 backend。
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
