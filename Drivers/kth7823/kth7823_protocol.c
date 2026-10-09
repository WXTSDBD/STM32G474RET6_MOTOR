/**
 * @file kth7823_protocol.c
 * @date 2026-10-08
 * @brief KTH7823 组帧、阻塞读角/寄存器和 unwrap。
 *
 * 阻塞读只给初始化与标定。热路径走异步单帧。
 */

#include "kth7823.h"

/**
 * @brief 拼读寄存器命令：opcode 01 + 6bit 地址 + 8bit 0。
 */
uint16_t kth7823_build_read_reg_cmd(uint8_t reg)
{
    return (uint16_t)(KTH7823_OPC_READ | (((uint16_t)reg & 0x3Fu) << 8));
}

static uint16_t kth7823_spi_xfer_word(KTH7823_HandleTypeDef *dev, uint16_t tx)
{
    uint16_t rx = 0U;

    KTH7823_CS_L(dev);
    if (HAL_SPI_TransmitReceive(dev->hspi, (uint8_t *)&tx, (uint8_t *)&rx, 1, 1000) != HAL_OK) {
        rx = 0U;
    }
    KTH7823_CS_H(dev);
    return rx;
}

/**
 * @brief 单帧阻塞读绝对角。MOSI 发 0。
 */
uint16_t kth7823_blocking_read_angle(KTH7823_HandleTypeDef *dev)
{
    if (dev == NULL || dev->hspi == NULL) {
        return 0U;
    }
    return kth7823_spi_xfer_word(dev, KTH7823_OPC_ANGLE);
}

/**
 * @brief 双帧阻塞读 8bit 寄存器（第二帧数据在高字节）。
 */
uint16_t kth7823_blocking_read_reg(KTH7823_HandleTypeDef *dev, uint8_t reg)
{
    uint16_t rsp;

    if (dev == NULL || dev->hspi == NULL) {
        return 0U;
    }

    (void)kth7823_spi_xfer_word(dev, kth7823_build_read_reg_cmd(reg));
    rsp = kth7823_spi_xfer_word(dev, KTH7823_OPC_ANGLE);
    return (uint16_t)((rsp >> 8) & 0xFFu);
}

/**
 * @brief 过零加减 2π，返回多圈机械角，单位 rad。
 */
float kth7823_unwrap(kth7823_ctx_t *ctx, uint16_t raw)
{
    int32_t d;

    if (ctx == NULL) {
        return 0.0f;
    }

    d = (int32_t)raw - (int32_t)ctx->raw_prev;
    if (d > (int32_t)KTH7823_UNWRAP_THRESH) {
        ctx->full_rotation_offset -= KTH7823_TWO_PI;
    } else if (d < -(int32_t)KTH7823_UNWRAP_THRESH) {
        ctx->full_rotation_offset += KTH7823_TWO_PI;
    }
    ctx->raw_prev = raw;

    return ctx->full_rotation_offset + (float)raw * KTH7823_ANGLE_SCALE;
}

/**
 * @brief 绑 SPI 和片选，片选拉高。
 */
void KTH7823_Init(KTH7823_HandleTypeDef *dev, SPI_HandleTypeDef *hspi,
                  GPIO_TypeDef *cs_port, uint16_t cs_pin)
{
    if (dev == NULL) {
        return;
    }

    dev->hspi = hspi;
    dev->cs_gpio_port = cs_port;
    dev->cs_gpio_pin = cs_pin;
    if (cs_port != NULL) {
        KTH7823_CS_H(dev);
    }
}
