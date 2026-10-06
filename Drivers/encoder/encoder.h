/**
 * @file encoder.h
 * @date 2026-10-06
 * @brief 编码器薄 API：踢一次 SPI、取 raw、换电角。
 *
 * kick 在电流环节拍开头调用。get_raw / get_theta_el 在同一拍稍后读。
 * 芯片协议和 DMA 不在本头。无感打开且编码器可选时，控制不要把 raw 当 Park。
 *
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
 */

#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

#include "encoder_spi_bus.h"

typedef struct encoder encoder_t;

/**
 * 编码器事件。KICK_DONE 发出命令；F1/F2 是双帧完成；BUSY 是本拍跳过；ERROR 是总线错。
 */
typedef enum {
    ENC_EVT_KICK_DONE = 0,
    ENC_EVT_F1_DONE,
    ENC_EVT_F2_DONE,
    ENC_EVT_KICK_SKIP_BUSY,
    ENC_EVT_ERROR,
} enc_event_t;

typedef struct {
    /** 事件类型。 */
    enc_event_t ev;
    /** 时戳，DWT 周期。 */
    uint32_t cyccnt;
    /** 附加量，含义随事件变。 */
    uint32_t aux;
} enc_profile_event_t;

typedef void (*encoder_profile_cb_t)(const encoder_t *e, const enc_profile_event_t *ev);

typedef struct {
    /** 阻塞初始化。 */
    int (*init)(encoder_t *e);
    /** 异步初始化。 */
    int (*async_init)(encoder_t *e);
    /** 踢一帧 SPI。 */
    void (*kick)(encoder_t *e);
    /** 读最新 raw。 */
    uint16_t (*get_raw)(const encoder_t *e);
    /** SPI 收完。 */
    void (*on_rx_complete)(encoder_t *e);
    /** SPI 出错。 */
    void (*on_error)(encoder_t *e);
    /** raw 展开成多圈机械角，单位 rad。 */
    float (*unwrap)(encoder_t *e, uint16_t raw);
} encoder_driver_t;

struct encoder {
    /** 芯片驱动。 */
    const encoder_driver_t *drv;
    /** 芯片私有上下文。 */
    void *chip_ctx;
    /** SPI 总线。 */
    encoder_spi_bus_t *bus;
    /** 可选：事件回调。 */
    encoder_profile_cb_t profile_cb;
    /** 电角偏置，单位 rad。 */
    float theta_el_offset_rad;
};

void encoder_init(encoder_t *e, const encoder_driver_t *drv, void *chip_ctx, encoder_spi_bus_t *bus);
void encoder_set_profile_cb(encoder_t *e, encoder_profile_cb_t cb);

int encoder_async_init(encoder_t *e);
void encoder_kick(encoder_t *e);
uint16_t encoder_get_raw(const encoder_t *e);
void encoder_on_spi_rx_complete(encoder_t *e);
void encoder_on_spi_error(encoder_t *e);
float encoder_get_angle(encoder_t *e, uint16_t raw);
float encoder_get_theta_el(const encoder_t *e, uint16_t raw, uint8_t pole_pairs, float offset_rad);

void encoder_set_theta_el_offset(encoder_t *e, float offset_rad);
float encoder_get_theta_el_offset(const encoder_t *e);

void encoder_profile_notify(const encoder_t *e, enc_event_t ev, uint32_t cyccnt, uint32_t aux);

#endif
