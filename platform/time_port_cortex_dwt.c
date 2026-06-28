/**
 * @file time_port_cortex_dwt.c
 * @brief Cortex-M DWT CYCCNT 时间 backend（非热路径）。
 */

#include "time_port.h"

#include "bsp_dwt.h"
#include "stm32g474xx.h"

void time_port_init(uint32_t cpu_mhz)
{
    DWT_Init(cpu_mhz);
}

uint32_t time_port_ms(void)
{
    return (uint32_t)DWT_GetTimeline_ms();
}

uint32_t time_port_cycles(void)
{
    return DWT->CYCCNT;
}
