/**
 * @file as5047.h
 * @date 2026-10-06
 * @brief AS5047 寄存器、句柄和 raw 换角。DMA 状态机在 as5047_async。
 *
 * 静态换角函数可在电流环节拍里调用。不要在这里踢 SPI。
 *
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
 */

#ifndef AS5047_H
#define AS5047_H

#include "spi.h"
#include "encoder.h"

#define AS5047_NOP         0x0000u
#define AS5047_ERRFL       0x0001u
#define AS5047_PROG        0x0003u
#define AS5047_DIAAGC      0x3FFCu
#define AS5047_MAG         0x3FFDu
#define AS5047_ANGLEUNC    0x3FFEu
#define AS5047_ANGLECOM    0x3FFFu

#define AS5047_RESOLUTION_BITS 14u
#define AS5047_RESOLUTION      (1u << AS5047_RESOLUTION_BITS)
#define AS5047_RAW_MASK        0x3FFFu
#define AS5047_UNWRAP_THRESH   12288u

#define AS5047_NOP_FRAME       (AS5047_NOP | 0x4000u)

#define AS5047_TWO_PI          6.28318530718f
#define AS5047_ANGLE_SCALE     0.00038349519f  /* 2*pi / 16384 */

typedef struct {
    /** HAL SPI 与片选。 */
    SPI_HandleTypeDef *hspi;
    /** 片选 GPIO 口。 */
    GPIO_TypeDef *cs_gpio_port;
    /** 片选引脚。 */
    uint16_t cs_gpio_pin;
    /** 上一圈机械角，unwrap 用。 */
    float angle_data_prev;
    /** 多圈偏置，单位 rad。 */
    float full_rotation_offset;
} AS5047_HandleTypeDef;

extern AS5047_HandleTypeDef AS5047_spi1_PORT;
extern AS5047_HandleTypeDef AS5047_spi3_PORT;

#define AS5047_CS_L(handle) \
    ((handle)->cs_gpio_port->BSRR = ((uint32_t)((handle)->cs_gpio_pin) << 16U))
#define AS5047_CS_H(handle) \
    ((handle)->cs_gpio_port->BSRR = (uint32_t)((handle)->cs_gpio_pin))

typedef enum {
    /** 空闲，可以踢下一帧。 */
    AS5047_PHASE_IDLE = 0,
    /** 第一帧已发出，等 DMA。 */
    AS5047_PHASE_FRAME1 = 1,
    /** 第二帧 NOP 已发出。 */
    AS5047_PHASE_FRAME2 = 2,
} as5047_phase_t;

/**
 * 异步双帧状态。ISR 写 raw，任务不要直改。
 */
typedef struct {
    /** 绑定的 HAL 句柄。 */
    AS5047_HandleTypeDef *hal;
    /** 本拍发出的读命令。 */
    uint16_t tx_cmd;
    /** 第二帧 NOP。 */
    uint16_t tx_nop;
    /** 最近一帧收到的字。 */
    uint16_t rx_buf;
    /** 最新角度 raw。 */
    volatile uint16_t raw;
    /** 双帧相位。 */
    volatile uint8_t phase;
    /** 上一拍 raw，unwrap 用。 */
    uint16_t raw_prev;
    /** 多圈偏置，单位 rad。 */
    float full_rotation_offset;
    /** 踢帧时的 DWT 周期。 */
    uint32_t kick_cyccnt;
    /** 第一帧完成相对踢帧的周期差。 */
    uint32_t f1_cb_delta;
    /** 连续因忙跳过的次数。过大则中止自愈。 */
    uint8_t skip_busy_n;
} as5047_ctx_t;

uint16_t as5047_parity_bit_calculate(uint16_t data);
uint16_t as5047_build_read_cmd(uint16_t reg);
uint16_t as5047_blocking_read(AS5047_HandleTypeDef *dev, uint16_t reg);
float as5047_unwrap(as5047_ctx_t *ctx, uint16_t raw);

void AS5047_Init(AS5047_HandleTypeDef *dev, SPI_HandleTypeDef *hspi,
                 GPIO_TypeDef *cs_port, uint16_t cs_pin);
uint16_t AS5047_read(AS5047_HandleTypeDef *dev, uint16_t reg);
float AS5047_GetAngle(AS5047_HandleTypeDef *dev);

/** Mechanical angle [0, 2pi) rad, single-turn from raw (no unwrap). */
static inline float as5047_raw_to_theta_mech(uint16_t raw, float offset_rad)
{
    float theta = (float)(raw & AS5047_RAW_MASK) * AS5047_ANGLE_SCALE + offset_rad;

    if (theta >= AS5047_TWO_PI) {
        theta -= AS5047_TWO_PI;
    } else if (theta < 0.0f) {
        theta += AS5047_TWO_PI;
    }
    return theta;
}

/** Electrical angle [0, 2pi) rad for SVPWM/Park @ 20kHz (no unwrap, no SPI). */
static inline float as5047_raw_to_theta_el(uint16_t raw, uint8_t pole_pairs, float offset_rad)
{
    uint32_t r = (uint32_t)(raw & AS5047_RAW_MASK) * (uint32_t)pole_pairs;

    r &= (AS5047_RESOLUTION - 1u);
    return as5047_raw_to_theta_mech((uint16_t)r, offset_rad);
}

extern const encoder_driver_t as5047_encoder_driver;

#endif
