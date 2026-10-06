/**
 * @file bsp_dwt.h
 * @date 2026-10-06
 * @brief 旧 DWT 计时 API。新代码优先 time_port。

 *
 * init 上电调用。Delay 会空转，不要在电流环里用。
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
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
