/**
 * @file adc_foc_port_stm32g4_reg.c
 * @date 2026-10-06
 * @brief 从 JDR 读三相电流并交给 adc_sample。

 *
 * 节拍限制见 adc_foc_port.h 文件头。
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
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
