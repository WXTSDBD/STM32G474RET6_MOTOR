/**
 * @file app_uart_dma_debug.h
 * @brief VOFA JustFloat 双缓冲 + LPUART DMA（ISR 写帧，RTOS 任务发送）。
 */

#ifndef APP_UART_DMA_DEBUG_H
#define APP_UART_DMA_DEBUG_H

#include <stdint.h>

#include "encoder.h"

void telem_encoder_profile_bind(encoder_t *e);

void telem_bringup_init(void);
void telem_bringup_tick(void);
void telem_bringup_try_send(void);

/** Keil Watch：g_telem_dbg；buf state 0=UNLOCKED 1=LOCKED 2=READY 3=SENDING */
typedef struct {
    volatile uint32_t isr_t0;
    volatile uint32_t isr_t1;
    volatile uint32_t cyccnt_end;
    volatile uint32_t isr_delta;
    /** FOC 核心（Clarke→Park→PI→SVPWM，不含观测器） */
    volatile uint32_t foc_delta;
    /** Veq/SMO 旁路观测器段 */
    volatile uint32_t obs_delta;
    volatile uint32_t ch1_wire;
    volatile uint32_t ch2_wire;

    volatile uint32_t tick_total;
    volatile uint32_t tick_decim_skip;
    volatile uint32_t tick_frame_ok;
    volatile uint32_t acquire_fail;
    volatile uint32_t seal_cnt;

    volatile uint32_t uart_task_loops;
    volatile uint32_t try_send_calls;
    volatile uint32_t dma_busy_skip;
    volatile uint32_t no_ready_skip;
    volatile uint32_t dma_start_ok;
    volatile uint32_t dma_start_fail;
    volatile uint32_t tx_cplt_cnt;

    volatile uint8_t  buf0_state;
    volatile uint8_t  buf1_state;
    volatile uint16_t buf0_used;
    volatile uint16_t buf1_used;
    volatile uint8_t  write_buf_active;
    volatile uint8_t  uart_gstate;
    volatile uint32_t uart_error;
    volatile uint16_t last_dma_bytes;

    volatile uint8_t  enc_dma_busy;
    volatile uint8_t  enc_dma_phase;
    volatile uint8_t  enc_spi_state;
    volatile uint32_t enc_kick_cnt;
    volatile uint32_t enc_chain_kick_cnt;
    volatile uint32_t enc_kick_skip_busy;
    volatile uint32_t enc_kick_skip_spi;
    volatile uint32_t enc_cplt_cnt;
    volatile uint32_t enc_err_cnt;
    volatile uint16_t enc_rx_word0;
    volatile uint16_t enc_rx_word1;
    volatile uint16_t enc_raw;
    volatile uint32_t enc_dma_kick_delta;
    volatile uint32_t enc_dma_f1_cb_delta;
    volatile uint32_t enc_dma_f2_cb_delta;
    volatile uint32_t enc_dma_cpu_delta;
    volatile uint32_t enc_dma_seq_delta;
    volatile uint32_t enc_dma_seq_delta_max;
    volatile uint32_t enc_total_delta;
} telem_dbg_t;

extern telem_dbg_t g_telem_dbg;

/** 调试 Watch：最近一次 isr_delta */
extern volatile uint32_t time_cnt;

#endif
