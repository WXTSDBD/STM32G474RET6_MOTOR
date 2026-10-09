/**
 * @file kth7823_async.c
 * @date 2026-10-08
 * @brief KTH7823 单帧 DMA 状态机。
 *
 * kick 从电流环节拍进。收完回调从 DMA 中断进。
 * 每拍一帧 MOSI=0 取角，无 AS5047 第二帧 NOP。
 */

#include "bringup_bench.h"
#include "kth7823.h"

#include "encoder_spi_bus.h"

static kth7823_ctx_t *kth7823_ctx(encoder_t *e)
{
    return (kth7823_ctx_t *)e->chip_ctx;
}

/**
 * @brief 读 DWT 周期计数。
 */
static uint32_t kth7823_cyccnt(void)
{
    return *(volatile uint32_t *)&DWT->CYCCNT;
}

static void kth7823_abort(encoder_t *e)
{
    kth7823_ctx_t *ctx = kth7823_ctx(e);

    if (ctx == NULL || e->bus == NULL) {
        return;
    }

    encoder_spi_bus_cs_high(e->bus);
    encoder_spi_bus_hw_stop(e->bus);
    ctx->phase = KTH7823_PHASE_IDLE;
}

/**
 * @brief 发出读角度的一帧。
 */
static int kth7823_kick_frame(encoder_t *e)
{
    kth7823_ctx_t *ctx = kth7823_ctx(e);
    uint32_t t0;
    uint32_t t1;

    if (ctx == NULL || e->bus == NULL) {
        return -1;
    }

    t0 = kth7823_cyccnt();
    encoder_spi_bus_cs_low(e->bus);
    if (encoder_spi_bus_start_word(e->bus, &ctx->tx_word, &ctx->rx_buf) != 0) {
        kth7823_abort(e);
        encoder_profile_notify(e, ENC_EVT_ERROR, kth7823_cyccnt(), 0U);
        return -1;
    }

    t1 = kth7823_cyccnt();
    ctx->kick_cyccnt = t1;
    ctx->phase = KTH7823_PHASE_FRAME1;
    encoder_profile_notify(e, ENC_EVT_KICK_DONE, t1, t1 - t0);
    return 0;
}

/**
 * @brief 填读角命令，相位回到空闲。
 */
static int kth7823_chip_init(encoder_t *e)
{
    kth7823_ctx_t *ctx = kth7823_ctx(e);

    if (ctx == NULL) {
        return -1;
    }

    ctx->tx_word = KTH7823_OPC_ANGLE;
    ctx->phase = KTH7823_PHASE_IDLE;
    ctx->raw = 0U;
    ctx->raw_prev = 0U;
    ctx->full_rotation_offset = 0.0f;
    ctx->kick_cyccnt = 0U;
    ctx->skip_busy_n = 0U;
    return 0;
}

/**
 * @brief 阻塞读一帧种子，再启动 DMA 硬件。
 */
static int kth7823_chip_async_init(encoder_t *e)
{
    kth7823_ctx_t *ctx = kth7823_ctx(e);

    if (ctx == NULL || ctx->hal == NULL || e->bus == NULL) {
        return -1;
    }

    ctx->raw = kth7823_raw_dir(kth7823_blocking_read_angle(ctx->hal));
    ctx->raw_prev = ctx->raw;
    encoder_spi_bus_hw_init(e->bus);
    return 0;
}

/**
 * @brief 空闲则踢一帧；连续忙则中止自愈。
 */
static void kth7823_chip_kick(encoder_t *e)
{
    kth7823_ctx_t *ctx = kth7823_ctx(e);

    if (ctx == NULL) {
        return;
    }

    if (ctx->phase != KTH7823_PHASE_IDLE) {
        if (ctx->skip_busy_n < 0xFFu) {
            ctx->skip_busy_n++;
        }
        if (ctx->skip_busy_n >= 3u) {
            kth7823_abort(e);
            ctx->skip_busy_n = 0u;
            encoder_profile_notify(e, ENC_EVT_ERROR, kth7823_cyccnt(), 4U);
        } else {
            encoder_profile_notify(e, ENC_EVT_KICK_SKIP_BUSY, kth7823_cyccnt(),
                                   (uint32_t)ctx->skip_busy_n);
        }
        return;
    }

    ctx->skip_busy_n = 0u;
    (void)kth7823_kick_frame(e);
}

/**
 * @brief 读最新 16 位角度 raw。
 */
static uint16_t kth7823_chip_get_raw(const encoder_t *e)
{
    const kth7823_ctx_t *ctx = (const kth7823_ctx_t *)e->chip_ctx;

    if (ctx == NULL) {
        return 0U;
    }
    return ctx->raw;
}

/**
 * @brief 单帧完成：锁存 raw，相位回空闲。
 * @note 复用 ENC_EVT_F2_DONE 表示「本拍采样完成」，便于旧遥测字段。
 */
static void kth7823_chip_on_rx_complete(encoder_t *e)
{
    kth7823_ctx_t *ctx = kth7823_ctx(e);
    uint32_t t0;
    uint32_t t1;

    if (ctx == NULL || e->bus == NULL) {
        return;
    }

    if (ctx->phase != KTH7823_PHASE_FRAME1) {
        return;
    }

    t0 = kth7823_cyccnt();
    encoder_spi_bus_cs_high(e->bus);
    ctx->raw = kth7823_raw_dir((uint16_t)(ctx->rx_buf & KTH7823_RAW_MASK));
    ctx->phase = KTH7823_PHASE_IDLE;
    t1 = kth7823_cyccnt();
    encoder_profile_notify(e, ENC_EVT_F2_DONE, t1, t1 - t0);
}

/**
 * @brief 出错则中止并通知。
 */
static void kth7823_chip_on_error(encoder_t *e)
{
    kth7823_abort(e);
    encoder_profile_notify(e, ENC_EVT_ERROR, kth7823_cyccnt(), 3U);
}

static float kth7823_chip_unwrap(encoder_t *e, uint16_t raw)
{
    return kth7823_unwrap(kth7823_ctx(e), raw);
}

/**
 * @brief 16bit raw 换电角。
 */
static float kth7823_chip_raw_to_theta_el(uint16_t raw, uint8_t pole_pairs, float offset_rad)
{
    return kth7823_raw_to_theta_el(raw, pole_pairs, offset_rad);
}

/**
 * @brief 阻塞读绝对角。
 */
static uint16_t kth7823_chip_blocking_read_angle(encoder_t *e)
{
    kth7823_ctx_t *ctx = kth7823_ctx(e);

    if (ctx == NULL || ctx->hal == NULL) {
        return 0U;
    }
    return kth7823_raw_dir(kth7823_blocking_read_angle(ctx->hal));
}

/**
 * @brief 1=单帧未结束。
 */
static uint8_t kth7823_chip_xfer_busy(const encoder_t *e)
{
    const kth7823_ctx_t *ctx = (const kth7823_ctx_t *)e->chip_ctx;

    if (ctx == NULL) {
        return 0U;
    }
    return (ctx->phase != (uint8_t)KTH7823_PHASE_IDLE) ? 1U : 0U;
}

const encoder_driver_t kth7823_encoder_driver = {
    .init = kth7823_chip_init,
    .async_init = kth7823_chip_async_init,
    .kick = kth7823_chip_kick,
    .get_raw = kth7823_chip_get_raw,
    .on_rx_complete = kth7823_chip_on_rx_complete,
    .on_error = kth7823_chip_on_error,
    .unwrap = kth7823_chip_unwrap,
    .raw_to_theta_el = kth7823_chip_raw_to_theta_el,
    .blocking_read_angle = kth7823_chip_blocking_read_angle,
    .xfer_busy = kth7823_chip_xfer_busy,
};
