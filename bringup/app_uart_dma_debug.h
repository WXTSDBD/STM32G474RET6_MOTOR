#ifndef APP_UART_DMA_DEBUG_H
#define APP_UART_DMA_DEBUG_H

#include <stdint.h>

/**
 * 双缓冲遥测：TIM1 ISR 写缓冲，RTOS 任务 DMA 发送。
 * g_telem_dbg：DWT 诊断，Keil Watch 直接看。
 */
void telem_bringup_init(void);
void telem_bringup_tick(void);
void telem_bringup_try_send(void);

typedef struct {
    volatile uint32_t isr_t0;     /**< 本拍 TIM1 ISR 入口 CYCCNT */
    volatile uint32_t isr_t1;     /**< 本拍 TIM1 ISR 出口 CYCCNT */
    volatile uint32_t cyccnt_end; /**< 同 isr_t1 */
    volatile uint32_t isr_delta;  /**< isr_t1 - isr_t0 */
    volatile uint32_t ch1_wire;   /**< 上一遥测帧 patch 的 ch1（cyccnt_end 快照） */
    volatile uint32_t ch2_wire;   /**< 上一遥测帧 patch 的 ch2（isr_delta） */
} telem_dbg_t;

extern telem_dbg_t g_telem_dbg;

/** 兼容旧 Watch：等于最近一次 ch2_wire（isr_delta） */
extern volatile uint32_t time_cnt;

#endif
