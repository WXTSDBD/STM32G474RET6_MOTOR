/**
 * @file encoder_spi_bus.h
 * @date 2026-10-06
 * @brief 编码器 SPI 总线：片选、踢一帧、DMA 完成。
 *
 * kick/start_word 从电流环节拍调用。dma_isr 只允许从对应 DMA 中断进。
 *
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
 */

#ifndef ENCODER_SPI_BUS_H
#define ENCODER_SPI_BUS_H

#include <stdint.h>

struct encoder_spi_bus;

typedef void (*encoder_spi_bus_cb_t)(struct encoder_spi_bus *bus);

typedef struct encoder_spi_bus_ops {
    /** 拉低片选。 */
    void (*cs_low)(struct encoder_spi_bus *bus);
    /** 拉高片选。 */
    void (*cs_high)(struct encoder_spi_bus *bus);
    /** 配置 SPI/DMA。 */
    void (*hw_init)(struct encoder_spi_bus *bus);
    /** 停 DMA，释放总线。 */
    void (*hw_stop)(struct encoder_spi_bus *bus);
    /**
     * 启动一帧收发。忙则返回非 0。
     * 形参按 16-bit 字（本编码器帧宽）；换非 16-bit 编码器时契约要改。
     */
    int (*start_word)(struct encoder_spi_bus *bus, const uint16_t *tx, uint16_t *rx);
    /** 不停 DMA 再发下一帧。 */
    int (*restart_word)(struct encoder_spi_bus *bus, const uint16_t *tx, uint16_t *rx);
    /** DMA 完成中断。 */
    void (*dma_isr)(struct encoder_spi_bus *bus);
} encoder_spi_bus_ops_t;

typedef struct encoder_spi_bus {
    /** 后端 ops。 */
    const encoder_spi_bus_ops_t *ops;
    /** SPI 外设。 */
    void *spi;
    /**
     * RX DMA 通道号（当前为 STM32 LL 通道枚举，芯片泄漏）。
     * 等第二块板再中立化为板级 desc 字段；本刀只标注。
     */
    uint32_t dma_ll_rx_ch;
    /**
     * TX DMA 通道号（同上，芯片泄漏）。
     */
    uint32_t dma_ll_tx_ch;
    /**
     * 片选 GPIO 口指针（当前为 GPIO_TypeDef*，芯片泄漏）。
     * 换厂商时契约要改；第二块板再抽。
     */
    void *cs_port;
    /** 片选引脚。 */
    uint16_t cs_pin;
    /** 1=DMA 进行中。 */
    volatile uint8_t busy;
    /** 绑定的编码器实例。 */
    void *user_ctx;
    /** 收完回调。 */
    encoder_spi_bus_cb_t on_rx_complete;
    /** 出错回调。 */
    encoder_spi_bus_cb_t on_error;
} encoder_spi_bus_t;

extern const encoder_spi_bus_ops_t encoder_spi_bus_ops_spi1_fast;
extern const encoder_spi_bus_ops_t encoder_spi_bus_ops_generic_g4;

void encoder_spi_bus_cs_low(encoder_spi_bus_t *bus);
void encoder_spi_bus_cs_high(encoder_spi_bus_t *bus);
void encoder_spi_bus_hw_init(encoder_spi_bus_t *bus);
void encoder_spi_bus_hw_stop(encoder_spi_bus_t *bus);
int encoder_spi_bus_start_word(encoder_spi_bus_t *bus, const uint16_t *tx, uint16_t *rx);
int encoder_spi_bus_restart_word(encoder_spi_bus_t *bus, const uint16_t *tx, uint16_t *rx);
uint8_t encoder_spi_bus_is_busy(const encoder_spi_bus_t *bus);
void encoder_spi_bus_dma_isr(encoder_spi_bus_t *bus);

#endif
