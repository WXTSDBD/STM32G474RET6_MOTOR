/**
 * @file encoder.c
 * @date 2026-10-06
 * @brief 编码器薄转发：把 kick/raw/unwrap/θ 转到芯片驱动。
 *
 * 节拍限制见 encoder.h 文件头。电角换算必须走 drv->raw_to_theta_el。
 *
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
 */

#include "encoder.h"

#include <stddef.h>

/**
 * @brief 绑驱动、芯片上下文和总线，并调用芯片 init。
 * @param e 编码器实例。不可为 NULL。
 */
void encoder_init(encoder_t *e, const encoder_driver_t *drv, void *chip_ctx, encoder_spi_bus_t *bus)
{
    e->drv = drv;
    e->chip_ctx = chip_ctx;
    e->bus = bus;
    e->profile_cb = NULL;
    e->theta_el_offset_rad = 0.0f;

    if (e->drv != NULL && e->drv->init != NULL) {
        (void)e->drv->init(e);
    }
}

/**
 * @brief 设置可选的事件回调。
 */
void encoder_set_profile_cb(encoder_t *e, encoder_profile_cb_t cb)
{
    e->profile_cb = cb;
}

/**
 * @brief 异步初始化。失败返回 -1。
 */
int encoder_async_init(encoder_t *e)
{
    if (e == NULL || e->drv == NULL || e->drv->async_init == NULL) {
        return -1;
    }
    return e->drv->async_init(e);
}

/**
 * @brief 踢一帧 SPI。e 或驱动为空则直接返回。
 */
void encoder_kick(encoder_t *e)
{
    if (e != NULL && e->drv != NULL && e->drv->kick != NULL) {
        e->drv->kick(e);
    }
}

/**
 * @brief 读最新角度 raw。无效时返回 0。
 */
uint16_t encoder_get_raw(const encoder_t *e)
{
    if (e == NULL || e->drv == NULL || e->drv->get_raw == NULL) {
        return 0U;
    }
    return e->drv->get_raw(e);
}

/**
 * @brief SPI 收完，转到芯片状态机。
 */
void encoder_on_spi_rx_complete(encoder_t *e)
{
    if (e != NULL && e->drv != NULL && e->drv->on_rx_complete != NULL) {
        e->drv->on_rx_complete(e);
    }
}

/**
 * @brief SPI 出错。
 */
void encoder_on_spi_error(encoder_t *e)
{
    if (e != NULL && e->drv != NULL && e->drv->on_error != NULL) {
        e->drv->on_error(e);
    }
}

/**
 * @brief raw 展开成多圈机械角，单位 rad。
 */
float encoder_get_angle(encoder_t *e, uint16_t raw)
{
    if (e == NULL || e->drv == NULL || e->drv->unwrap == NULL) {
        return 0.0f;
    }
    return e->drv->unwrap(e, raw);
}

/**
 * @brief 单圈电角 [0, 2π)，给 Park/SVPWM。不做 unwrap、不踢 SPI。
 */
float encoder_get_theta_el(const encoder_t *e, uint16_t raw, uint8_t pole_pairs, float offset_rad)
{
    if (e == NULL || e->drv == NULL || e->drv->raw_to_theta_el == NULL) {
        return 0.0f;
    }
    return e->drv->raw_to_theta_el(raw, pole_pairs, offset_rad);
}

/**
 * @brief 阻塞读一角 raw。驱动未实现时返回 0。
 */
uint16_t encoder_blocking_read_angle(encoder_t *e)
{
    if (e == NULL || e->drv == NULL || e->drv->blocking_read_angle == NULL) {
        return 0U;
    }
    return e->drv->blocking_read_angle(e);
}

/**
 * @brief 1=传输未完成。
 */
uint8_t encoder_xfer_busy(const encoder_t *e)
{
    if (e == NULL || e->drv == NULL || e->drv->xfer_busy == NULL) {
        return 0U;
    }
    return e->drv->xfer_busy(e);
}

/**
 * @brief 写电角偏置，单位 rad。
 */
void encoder_set_theta_el_offset(encoder_t *e, float offset_rad)
{
    if (e != NULL) {
        e->theta_el_offset_rad = offset_rad;
    }
}

/**
 * @brief 读电角偏置，单位 rad。
 */
float encoder_get_theta_el_offset(const encoder_t *e)
{
    if (e == NULL) {
        return 0.0f;
    }
    return e->theta_el_offset_rad;
}

/**
 * @brief 若有回调则上报一次事件。
 */
void encoder_profile_notify(const encoder_t *e, enc_event_t ev, uint32_t cyccnt, uint32_t aux)
{
    enc_profile_event_t pev;

    if (e == NULL || e->profile_cb == NULL) {
        return;
    }

    pev.ev = ev;
    pev.cyccnt = cyccnt;
    pev.aux = aux;
    e->profile_cb(e, &pev);
}
