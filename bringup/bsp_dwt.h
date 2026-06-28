/**
 ******************************************************************************
 * @file    bsp_dwt.h
 * @brief   DWT 高精度计时（legacy API；新代码优先 time_port_*）。
 ******************************************************************************
 */

#ifndef BSP_DWT_H
#define BSP_DWT_H

#include <stdint.h>

#include "stm32g474xx.h"

typedef struct {
    uint32_t s;
    uint16_t ms;
    uint16_t us;
} DWT_Time_t;

void DWT_Init(uint32_t CPU_Freq_mHz);
float DWT_GetDeltaT(uint32_t *cnt_last);
double DWT_GetDeltaT64(uint32_t *cnt_last);
float DWT_GetTimeline_s(void);
float DWT_GetTimeline_ms(void);
uint64_t DWT_GetTimeline_us(void);
void DWT_Delay(float Delay);
void DWT_SysTimeUpdate(void);
void DWT_Delay_us(uint32_t us);

#endif
