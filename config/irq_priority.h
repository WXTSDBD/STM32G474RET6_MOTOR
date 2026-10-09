/**
 * @file irq_priority.h
 * @date 2026-10-07
 * @brief 业务中断抢占优先级表（数值越小越高）。
 *
 * CubeMX 生成的 MX_*_Init 仍会写一版 NVIC；上电后由 irq_priority_apply()
 * 再写一遍，以本表为准。不要手改 Core/Src/dma.c 里的业务注释优先级。
 */

#ifndef CONFIG_IRQ_PRIORITY_H
#define CONFIG_IRQ_PRIORITY_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** SPI1 RX/TX DMA：须能抢占 ADC FOC。 */
#define IRQ_PRIO_SPI1_DMA       1u
/** SPI3 编码器 RX DMA：与 SPI1 同档。 */
#define IRQ_PRIO_SPI3_DMA       1u
/** TIM 刹车 / Update（与现 CubeMX 一致）。 */
#define IRQ_PRIO_TIM_BRK_UP     1u
/** ADC1/2/3/5 注入完成（FOC 节拍）。 */
#define IRQ_PRIO_ADC_FOC        2u
/** FDCAN。 */
#define IRQ_PRIO_FDCAN          3u
/** UART / LPUART 及同档 DMA。 */
#define IRQ_PRIO_UART           4u
/** 其余 DMA 通道。 */
#define IRQ_PRIO_DMA_LO         5u

void irq_priority_apply(void);

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_IRQ_PRIORITY_H */
