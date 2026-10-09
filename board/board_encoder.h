/**
 * @file board_encoder.h
 * @date 2026-10-06
 * @brief 板级编码器实例。M1 默认可绑 SPI1/AS5047 或 SPI3/KTH7823。
 *
 * dma_isr 只进对应编码器 DMA 中断。init 在上电调用。
 * 选源宏见 bringup_bench.h 的 M1_ENCODER_SRC。
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
 */

#ifndef BOARD_ENCODER_H
#define BOARD_ENCODER_H

#include "encoder.h"
#include "encoder_spi_bus.h"

extern encoder_t enc_m1;
extern encoder_spi_bus_t enc_m1_bus;

extern encoder_t enc_kth_spi3;
extern encoder_spi_bus_t enc_kth_spi3_bus;

/** 占位：第二轴语义，单轴阶段可不 init。 */
extern encoder_t enc_m2;

void board_encoder_m1_init(void);
void board_encoder_m1_dma_isr(void);

void board_encoder_spi3_init(void);
void board_encoder_spi3_dma_isr(void);

/** 按 M1_ENCODER_SRC 初始化并返回 M1 角源实例。 */
encoder_t *board_encoder_m1_setup(void);

#endif
