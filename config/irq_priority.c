/**
 * @file irq_priority.c
 * @date 2026-10-07
 * @brief 在全部 MX_*_Init 之后重写业务 NVIC 优先级。
 *
 * 数值与当前 CubeMX 生成值对齐，本刀不改抢占关系，只收拢真相源。
 */

#include "irq_priority.h"

#include "stm32g4xx_hal.h"

/**
 * @brief 按 irq_priority.h 重写关键 IRQ 抢占优先级。
 * @note 须在 MX_DMA_Init / MX_ADC*_Init / MX_TIM*_Init 等之后调用。
 */
void irq_priority_apply(void)
{
    /* DMA：SPI1 高、UART 相关中、其余低（与 dma.c 现网一致）。 */
    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, IRQ_PRIO_UART, 0u);
    HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, IRQ_PRIO_SPI1_DMA, 0u);
    HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, IRQ_PRIO_SPI1_DMA, 0u);
    HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, IRQ_PRIO_DMA_LO, 0u);
    HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, IRQ_PRIO_DMA_LO, 0u);
    HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, IRQ_PRIO_SPI3_DMA, 0u);
    HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, IRQ_PRIO_SPI3_DMA, 0u);

    /* ADC FOC 节拍。 */
    HAL_NVIC_SetPriority(ADC1_2_IRQn, IRQ_PRIO_ADC_FOC, 0u);
    HAL_NVIC_SetPriority(ADC3_IRQn, IRQ_PRIO_ADC_FOC, 0u);
    HAL_NVIC_SetPriority(ADC5_IRQn, IRQ_PRIO_ADC_FOC, 0u);

    /* TIM 刹车 / Update（与 tim.c 现网一致）。 */
    HAL_NVIC_SetPriority(TIM1_BRK_TIM15_IRQn, IRQ_PRIO_TIM_BRK_UP, 0u);
    HAL_NVIC_SetPriority(TIM1_UP_TIM16_IRQn, IRQ_PRIO_TIM_BRK_UP, 0u);
    HAL_NVIC_SetPriority(TIM8_BRK_IRQn, IRQ_PRIO_TIM_BRK_UP, 0u);
    HAL_NVIC_SetPriority(TIM8_UP_IRQn, IRQ_PRIO_TIM_BRK_UP, 0u);

    /* 通信。 */
    HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, IRQ_PRIO_FDCAN, 0u);
    HAL_NVIC_SetPriority(LPUART1_IRQn, IRQ_PRIO_UART, 0u);
    HAL_NVIC_SetPriority(USART1_IRQn, IRQ_PRIO_UART, 0u);
}
