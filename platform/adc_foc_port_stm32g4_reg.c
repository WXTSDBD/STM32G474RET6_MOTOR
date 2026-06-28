/**
 * @file adc_foc_port_stm32g4_reg.c
 * @brief STM32G4 FOC ADC JEOC backend：直读 JDR，热路径无 LL API。
 */

#include "adc_foc_port.h"

#include <stddef.h>

static void adc_foc_port_stm32g4_on_jeoc(adc_foc_port_t *port,
                                         adc_sample_t *sample,
                                         void *hadc)
{
    (void)port;
    adc_sample_jeoc_foc(sample, hadc);
}

const adc_foc_port_ops_t adc_foc_port_ops_stm32g4_reg = {
    .on_jeoc = adc_foc_port_stm32g4_on_jeoc,
};

void adc_foc_port_on_jeoc(adc_foc_port_t *port, adc_sample_t *sample, void *hadc)
{
    if (port == NULL || sample == NULL || hadc == NULL ||
        port->ops == NULL || port->ops->on_jeoc == NULL) {
        return;
    }
    port->ops->on_jeoc(port, sample, hadc);
}
