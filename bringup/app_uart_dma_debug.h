#ifndef APP_UART_DMA_DEBUG_H
#define APP_UART_DMA_DEBUG_H

#include <stdint.h>

/**
 * ????????TIM1 ISR ???????RTOS ???? DMA ?????
 * Keil Watch ???? g_telem_dbg ???????
 *
 * ???? state??0=UNLOCKED 1=LOCKED 2=READY 3=SENDING
 * uart_gstate??HAL ????0x20=READY 0x21=BUSY_TX
 */
void telem_bringup_init(void);
void telem_bringup_tick(void);
void telem_bringup_try_send(void);

typedef struct {
    /* --- ISR ?????DWT??--- */
    volatile uint32_t isr_t0;
    volatile uint32_t isr_t1;
    volatile uint32_t cyccnt_end;
    volatile uint32_t isr_delta;
    volatile uint32_t ch1_wire;
    volatile uint32_t ch2_wire;

    /* --- ISR ?????? --- */
    volatile uint32_t tick_total;       /**< telem_bringup_tick ?????? */
    volatile uint32_t tick_decim_skip;  /**< ??????????????? return */
    volatile uint32_t tick_frame_ok;    /**< ??????? 1 ??? */
    volatile uint32_t acquire_fail;     /**< ?????? UNLOCKED ???? */
    volatile uint32_t seal_cnt;         /**< ??? READY ???? */

    /* --- RTOS ???????? --- */
    volatile uint32_t uart_task_loops;  /**< UART_DMA_DEBUG_TASK ??????? */
    volatile uint32_t try_send_calls;   /**< telem_bringup_try_send ??? */
    volatile uint32_t dma_busy_skip;    /**< UART ?? BUSY_TX?????? */
    volatile uint32_t no_ready_skip;    /**< ?? READY ????????? */
    volatile uint32_t dma_start_ok;     /**< HAL_UART_Transmit_DMA ??? */
    volatile uint32_t dma_start_fail;   /**< HAL_UART_Transmit_DMA ??? */
    volatile uint32_t tx_cplt_cnt;      /**< TxCplt ??????? */

    /* --- ???????????? try_send ?????--- */
    volatile uint8_t  buf0_state;
    volatile uint8_t  buf1_state;
    volatile uint16_t buf0_used;
    volatile uint16_t buf1_used;
    volatile uint8_t  write_buf_active; /**< s_write_buf ???=1 */
    volatile uint8_t  uart_gstate;      /**< hlpuart1.gState */
    volatile uint32_t uart_error;      /**< hlpuart1.ErrorCode */
    volatile uint16_t last_dma_bytes;   /**< ?????? DMA ????????? */

    /* --- AS5047 SPI1 DMA??Keil Watch: g_telem_dbg.enc_*??--- */
    volatile uint8_t  enc_dma_busy;       /**< 1=SPI DMA ?????? */
    volatile uint8_t  enc_dma_phase;      /**< 0=IDLE 1=FRAME1 2=FRAME2 */
    volatile uint8_t  enc_spi_state;      /**< hspi1.State??0x01=READY */
    volatile uint32_t enc_kick_cnt;       /**< FRAME1 kick starts (chain + manual DmaKick) */
    volatile uint32_t enc_chain_kick_cnt; /**< kicks from DMA chain only */
    volatile uint32_t enc_kick_skip_busy; /**< manual DmaKick while busy (TIM1 no longer kicks) */
    volatile uint32_t enc_kick_skip_spi;  /**< kick when SPI not READY */
    volatile uint32_t enc_cplt_cnt;       /**< TxRxCplt ?????? */
    volatile uint32_t enc_err_cnt;        /**< SPI Error ??????? */
    volatile uint16_t enc_rx_word0;       /**< CMD frame rx */
    volatile uint16_t enc_rx_word1;       /**< NOP frame rx */
    volatile uint16_t enc_raw;            /**< ????????????? 14bit raw */
    /* DWT cycle???????? DMA ??????TIM1 ??Keil Watch ?? isr_delta ????????? */
    volatile uint32_t enc_dma_kick_delta; /**< FRAME1 kick CPU cycle (DMA chain) */
    volatile uint32_t enc_dma_f1_cb_delta; /**< FRAME1 TxRxCplt ??? CPU cycle */
    volatile uint32_t enc_dma_f2_cb_delta; /**< FRAME2 TxRxCplt ??? CPU cycle */
    volatile uint32_t enc_dma_cpu_delta;   /**< ???? f1_cb + f2_cb ?? CPU */
    volatile uint32_t enc_dma_seq_delta;   /**< kick -> FRAME2 done wall cycle */
    volatile uint32_t enc_dma_seq_delta_max; /**< max enc_dma_seq_delta since init */
    volatile uint32_t enc_total_delta;     /**< isr_delta + enc_dma_cpu_delta reference */
} telem_dbg_t;

extern telem_dbg_t g_telem_dbg;

/** ????? Watch???????? ch2_wire??isr_delta?? */
extern volatile uint32_t time_cnt;

#endif
