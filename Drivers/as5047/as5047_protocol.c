/**
 * @file as5047_protocol.c
 * @date 2026-10-06
 * @brief AS5047 奇偶、读命令、阻塞读和 unwrap。

 *
 * 阻塞读只给初始化用。热路径走异步双帧。
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
 */

#include "as5047.h"

#include "gpio.h"

#define AS5047_TWO_PI_F        AS5047_TWO_PI
#define AS5047_ANGLE_SCALE_F   AS5047_ANGLE_SCALE

/**
 * @brief 偶校验位。
 */
uint16_t as5047_parity_bit_calculate(uint16_t data)
{
    uint16_t parity = 0U;

    while (data != 0U) {
        parity ^= data;
        data >>= 1;
    }
    return (uint16_t)(parity & 0x1U);
}

/**
 * @brief 拼读命令并补校验。
 */
uint16_t as5047_build_read_cmd(uint16_t reg)
{
    reg |= 0x4000u;
    if (as5047_parity_bit_calculate(reg) == 1U) {
        reg |= 0x8000u;
    }
    return reg;
}

static uint16_t as5047_spi_xfer_word(AS5047_HandleTypeDef *dev, uint16_t tx)
{
    uint16_t rx = 0U;

    AS5047_CS_L(dev);
    if (HAL_SPI_TransmitReceive(dev->hspi, (uint8_t *)&tx, (uint8_t *)&rx, 1, 1000) != HAL_OK) {
        rx = 0U;
    }
    AS5047_CS_H(dev);
    return rx;
}

/**
 * @brief 双帧阻塞读寄存器。只给初始化。
 */
uint16_t as5047_blocking_read(AS5047_HandleTypeDef *dev, uint16_t reg)
{
    uint16_t data;

    (void)as5047_spi_xfer_word(dev, as5047_build_read_cmd(reg));
    data = as5047_spi_xfer_word(dev, AS5047_NOP_FRAME);
    return (uint16_t)(data & AS5047_RAW_MASK);
}

/**
 * @brief 过零加减 2π，返回多圈机械角，单位 rad。
 */
float as5047_unwrap(as5047_ctx_t *ctx, uint16_t raw)
{
    int32_t d;

    if (ctx == NULL) {
        return 0.0f;
    }

    d = (int32_t)raw - (int32_t)ctx->raw_prev;
    if (d > (int32_t)AS5047_UNWRAP_THRESH) {
        ctx->full_rotation_offset -= AS5047_TWO_PI_F;
    } else if (d < -(int32_t)AS5047_UNWRAP_THRESH) {
        ctx->full_rotation_offset += AS5047_TWO_PI_F;
    }
    ctx->raw_prev = raw;

    return ctx->full_rotation_offset + (float)raw * AS5047_ANGLE_SCALE_F;
}

AS5047_HandleTypeDef AS5047_spi1_PORT;
AS5047_HandleTypeDef AS5047_spi3_PORT;

/**
 * @brief 绑 SPI 和片选，片选拉高。
 */
void AS5047_Init(AS5047_HandleTypeDef *dev, SPI_HandleTypeDef *hspi,
                 GPIO_TypeDef *cs_port, uint16_t cs_pin)
{
    dev->hspi = hspi;
    dev->cs_gpio_port = cs_port;
    dev->cs_gpio_pin = cs_pin;
    dev->angle_data_prev = 0.0f;
    dev->full_rotation_offset = 0.0f;
    AS5047_CS_H(dev);
}

/**
 * @brief 阻塞读。热路径不要用。
 */
uint16_t AS5047_read(AS5047_HandleTypeDef *dev, uint16_t reg)
{
    return as5047_blocking_read(dev, reg);
}

float AS5047_GetAngle(AS5047_HandleTypeDef *dev)
{
    float angle_data = (float)AS5047_read(dev, AS5047_ANGLEUNC);
    float d_angle = angle_data - dev->angle_data_prev;

    if (d_angle > 0.0f) {
        if (d_angle > (float)AS5047_UNWRAP_THRESH) {
            dev->full_rotation_offset -= AS5047_TWO_PI_F;
        }
    } else if (d_angle < -(float)AS5047_UNWRAP_THRESH) {
        dev->full_rotation_offset += AS5047_TWO_PI_F;
    }

    dev->angle_data_prev = angle_data;
    return dev->full_rotation_offset + angle_data * AS5047_ANGLE_SCALE_F;
}
