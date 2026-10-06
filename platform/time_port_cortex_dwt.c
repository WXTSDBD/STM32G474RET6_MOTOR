/**
 * @file time_port_cortex_dwt.c
 * @date 2026-10-06
 * @brief 用 DWT 实现 time_port。

 *
 * 节拍限制见 time_port.h 文件头。
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
 */

#include "time_port.h"

#include "bsp_dwt.h"
#include "stm32g474xx.h"

void time_port_init(uint32_t cpu_mhz)
{
    DWT_Init(cpu_mhz);
}

/**
 * @brief 上电后的毫秒。
 */
uint32_t time_port_ms(void)
{
    return (uint32_t)DWT_GetTimeline_ms();
}

uint32_t time_port_cycles(void)
{
    return DWT->CYCCNT;
}
