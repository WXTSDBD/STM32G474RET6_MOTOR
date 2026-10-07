/**
 * @file board_encoder.h
 * @date 2026-10-06
 * @brief 板级编码器实例。M1 已绑 DMA，M2 占位。

 *
 * dma_isr 只进 M1 编码器 DMA 中断。init 在上电调用。
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
 */

#ifndef BOARD_ENCODER_H
#define BOARD_ENCODER_H

#include "encoder.h"
#include "encoder_spi_bus.h"
#include "as5047.h"

extern encoder_t enc_m1;
extern encoder_spi_bus_t enc_m1_bus;
extern as5047_ctx_t enc_m1_as5047;

void board_encoder_m1_init(void);
void board_encoder_m1_dma_isr(void);

/*
 * M2 占位：硬件为 SPI3 + PA15（第二颗 AS5047），双路 FOC 时再接 DMA 总线。
 * 单轴阶段不 init；main 只把 SPI3_CS 拉高。
 */
extern encoder_t enc_m2;

#endif
