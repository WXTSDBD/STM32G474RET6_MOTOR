#ifndef APP_UART_DMA_DEBUG_H
#define APP_UART_DMA_DEBUG_H

/**
 * 双缓冲遥测：写缓冲（tick）与 DMA 发送（try_send）分离。
 * 当前由 UART_DMA_DEBUG_TASK 调用；后期可在 TIM1 ISR 中仅调用 telem_bringup_tick()。
 */
void telem_bringup_tick(void);
void telem_bringup_try_send(void);

#endif
