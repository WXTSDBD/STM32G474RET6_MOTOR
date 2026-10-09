/**
 * @file kth7823.h
 * @date 2026-10-08
 * @brief KTH7823 寄存器、句柄和 raw 换角。DMA 状态机在 kth7823_async。
 *
 * 静态换角可在电流环节拍调用。不要在这里踢 SPI。
 * SPI 须 Mode3；热路径读角为单帧 MOSI=0。
 */

#ifndef KTH7823_H
#define KTH7823_H

#include "spi.h"
#include "encoder.h"

#define KTH7823_REG_Z_L        0x00u
#define KTH7823_REG_Z_H        0x01u
#define KTH7823_REG_RD         0x09u

#define KTH7823_OPC_ANGLE      0x0000u
#define KTH7823_OPC_READ       0x4000u
#define KTH7823_OPC_WRITE      0x8000u

#define KTH7823_RESOLUTION_BITS 16u
#define KTH7823_RESOLUTION      (1u << KTH7823_RESOLUTION_BITS)
#define KTH7823_RAW_MASK        0xFFFFu
#define KTH7823_UNWRAP_THRESH   24576u

#define KTH7823_TWO_PI          6.28318530718f
#define KTH7823_ANGLE_SCALE     9.587379924285257e-5f  /* 2*pi / 65536 */

/**
 * 1=锁存/阻塞读后 raw = -raw（16bit）。须在包含本头之前由台架宏定义；
 * 默认 0。与 M1_ENCODER_DIR_REV 对齐。
 */
#ifndef KTH7823_DIR_REV
#ifdef M1_ENCODER_DIR_REV
#define KTH7823_DIR_REV         M1_ENCODER_DIR_REV
#else
#define KTH7823_DIR_REV         0
#endif
#endif

/** 按安装方向规范化 raw（电角与 unwrap 共用）。 */
static inline uint16_t kth7823_raw_dir(uint16_t raw)
{
#if KTH7823_DIR_REV
    return (uint16_t)(0u - (raw & KTH7823_RAW_MASK));
#else
    return (uint16_t)(raw & KTH7823_RAW_MASK);
#endif
}

typedef struct {
    /** HAL SPI 与片选。 */
    SPI_HandleTypeDef *hspi;
    /** 片选 GPIO 口。 */
    GPIO_TypeDef *cs_gpio_port;
    /** 片选引脚。 */
    uint16_t cs_gpio_pin;
} KTH7823_HandleTypeDef;

#define KTH7823_CS_L(handle) \
    ((handle)->cs_gpio_port->BSRR = ((uint32_t)((handle)->cs_gpio_pin) << 16U))
#define KTH7823_CS_H(handle) \
    ((handle)->cs_gpio_port->BSRR = (uint32_t)((handle)->cs_gpio_pin))

typedef enum {
    /** 空闲，可以踢下一帧。 */
    KTH7823_PHASE_IDLE = 0,
    /** 角度帧已发出，等 DMA。 */
    KTH7823_PHASE_FRAME1 = 1,
} kth7823_phase_t;

/**
 * 异步单帧状态。ISR 写 raw，任务不要直改。
 */
typedef struct {
    /** 绑定的 HAL 句柄（阻塞读用）。 */
    KTH7823_HandleTypeDef *hal;
    /** 本拍发出的字（角度读为 0）。 */
    uint16_t tx_word;
    /** 最近一帧收到的字。 */
    uint16_t rx_buf;
    /** 最新角度 raw。 */
    volatile uint16_t raw;
    /** 单帧相位。 */
    volatile uint8_t phase;
    /** 上一拍 raw，unwrap 用。 */
    uint16_t raw_prev;
    /** 多圈偏置，单位 rad。 */
    float full_rotation_offset;
    /** 踢帧时的 DWT 周期。 */
    uint32_t kick_cyccnt;
    /** 连续因忙跳过的次数。过大则中止自愈。 */
    uint8_t skip_busy_n;
} kth7823_ctx_t;

uint16_t kth7823_build_read_reg_cmd(uint8_t reg);
uint16_t kth7823_blocking_read_angle(KTH7823_HandleTypeDef *dev);
uint16_t kth7823_blocking_read_reg(KTH7823_HandleTypeDef *dev, uint8_t reg);
float kth7823_unwrap(kth7823_ctx_t *ctx, uint16_t raw);

void KTH7823_Init(KTH7823_HandleTypeDef *dev, SPI_HandleTypeDef *hspi,
                  GPIO_TypeDef *cs_port, uint16_t cs_pin);

/** Mechanical angle [0, 2pi) rad, single-turn from raw (no unwrap). */
static inline float kth7823_raw_to_theta_mech(uint16_t raw, float offset_rad)
{
    float theta = (float)(raw & KTH7823_RAW_MASK) * KTH7823_ANGLE_SCALE + offset_rad;

    if (theta >= KTH7823_TWO_PI) {
        theta -= KTH7823_TWO_PI;
    } else if (theta < 0.0f) {
        theta += KTH7823_TWO_PI;
    }
    return theta;
}

/** Electrical angle [0, 2pi) rad for SVPWM/Park (no unwrap, no SPI). */
static inline float kth7823_raw_to_theta_el(uint16_t raw, uint8_t pole_pairs, float offset_rad)
{
    uint32_t r = (uint32_t)(raw & KTH7823_RAW_MASK) * (uint32_t)pole_pairs;

    r &= (KTH7823_RESOLUTION - 1u);
    return kth7823_raw_to_theta_mech((uint16_t)r, offset_rad);
}

extern const encoder_driver_t kth7823_encoder_driver;

#endif
