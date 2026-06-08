#ifndef APP_UART_DMA_DEBUG_H
#define APP_UART_DMA_DEBUG_H

#include <stdint.h>

/**
 * 双缓冲遥测：TIM1 ISR 写缓冲，RTOS 任务 DMA 发送。
 * Keil Watch 添加 g_telem_dbg 展开查看。
 *
 * 缓冲 state：0=UNLOCKED 1=LOCKED 2=READY 3=SENDING
 * uart_gstate：HAL 状态，0x20=READY 0x21=BUSY_TX
 */
void telem_bringup_init(void);
void telem_bringup_tick(void);
void telem_bringup_try_send(void);

typedef struct {
    /* --- ISR 计时（DWT）--- */
    volatile uint32_t isr_t0;
    volatile uint32_t isr_t1;
    volatile uint32_t cyccnt_end;
    volatile uint32_t isr_delta;
    volatile uint32_t ch1_wire;
    volatile uint32_t ch2_wire;

    /* --- ISR 写缓冲 --- */
    volatile uint32_t tick_total;       /**< telem_bringup_tick 入口次数 */
    volatile uint32_t tick_decim_skip;  /**< 降采样未到，直接 return */
    volatile uint32_t tick_frame_ok;    /**< 成功写入 1 小帧 */
    volatile uint32_t acquire_fail;     /**< 拿不到 UNLOCKED 缓冲 */
    volatile uint32_t seal_cnt;         /**< 封包 READY 次数 */

    /* --- RTOS 发送任务 --- */
    volatile uint32_t uart_task_loops;  /**< UART_DMA_DEBUG_TASK 循环次数 */
    volatile uint32_t try_send_calls;   /**< telem_bringup_try_send 入口 */
    volatile uint32_t dma_busy_skip;    /**< UART 仍 BUSY_TX，跳过 */
    volatile uint32_t no_ready_skip;    /**< 无 READY 缓冲，跳过 */
    volatile uint32_t dma_start_ok;     /**< HAL_UART_Transmit_DMA 成功 */
    volatile uint32_t dma_start_fail;   /**< HAL_UART_Transmit_DMA 失败 */
    volatile uint32_t tx_cplt_cnt;      /**< TxCplt 回调次数 */

    /* --- 双缓冲快照（每次 try_send 更新）--- */
    volatile uint8_t  buf0_state;
    volatile uint8_t  buf1_state;
    volatile uint16_t buf0_used;
    volatile uint16_t buf1_used;
    volatile uint8_t  write_buf_active; /**< s_write_buf 非空=1 */
    volatile uint8_t  uart_gstate;      /**< hlpuart1.gState */
    volatile uint32_t uart_error;      /**< hlpuart1.ErrorCode */
    volatile uint16_t last_dma_bytes;   /**< 最近一次 DMA 发送字节数 */
} telem_dbg_t;

extern telem_dbg_t g_telem_dbg;

/** 兼容旧 Watch：最近一次 ch2_wire（isr_delta） */
extern volatile uint32_t time_cnt;

#endif
