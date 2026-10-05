#include "as5047.h"

#include "encoder_spi_bus.h"

static as5047_ctx_t *as5047_ctx(encoder_t *e)
{
    return (as5047_ctx_t *)e->chip_ctx;
}

static uint32_t as5047_cyccnt(void)
{
    return *(volatile uint32_t *)&DWT->CYCCNT;
}

static void as5047_abort(encoder_t *e)
{
    as5047_ctx_t *ctx = as5047_ctx(e);

    if (ctx == NULL || e->bus == NULL) {
        return;
    }

    encoder_spi_bus_cs_high(e->bus);
    encoder_spi_bus_hw_stop(e->bus);
    ctx->phase = AS5047_PHASE_IDLE;
}

static int as5047_kick_frame1(encoder_t *e)
{
    as5047_ctx_t *ctx = as5047_ctx(e);
    uint32_t t0;
    uint32_t t1;

    if (ctx == NULL || e->bus == NULL) {
        return -1;
    }

    t0 = as5047_cyccnt();
    encoder_spi_bus_cs_low(e->bus);
    if (encoder_spi_bus_start_word(e->bus, &ctx->tx_cmd, &ctx->rx_buf) != 0) {
        as5047_abort(e);
        encoder_profile_notify(e, ENC_EVT_ERROR, as5047_cyccnt(), 0U);
        return -1;
    }

    t1 = as5047_cyccnt();
    ctx->kick_cyccnt = t1;
    ctx->phase = AS5047_PHASE_FRAME1;
    encoder_profile_notify(e, ENC_EVT_KICK_DONE, t1, t1 - t0);
    return 0;
}

static int as5047_chip_init(encoder_t *e)
{
    as5047_ctx_t *ctx = as5047_ctx(e);

    if (ctx == NULL) {
        return -1;
    }

    ctx->tx_cmd = as5047_build_read_cmd(AS5047_ANGLEUNC);
    ctx->tx_nop = AS5047_NOP_FRAME;
    ctx->phase = AS5047_PHASE_IDLE;
    ctx->raw = 0U;
    ctx->raw_prev = 0U;
    ctx->full_rotation_offset = 0.0f;
    ctx->kick_cyccnt = 0U;
    ctx->f1_cb_delta = 0U;
    ctx->skip_busy_n = 0U;
    return 0;
}

static int as5047_chip_async_init(encoder_t *e)
{
    as5047_ctx_t *ctx = as5047_ctx(e);

    if (ctx == NULL || ctx->hal == NULL || e->bus == NULL) {
        return -1;
    }

    ctx->raw = as5047_blocking_read(ctx->hal, AS5047_ANGLEUNC);
    ctx->raw_prev = ctx->raw;
    encoder_spi_bus_hw_init(e->bus);
    return 0;
}

static void as5047_chip_kick(encoder_t *e)
{
    as5047_ctx_t *ctx = as5047_ctx(e);

    if (ctx == NULL) {
        return;
    }

    if (ctx->phase != AS5047_PHASE_IDLE) {
        /*
         * FOC ISR 过长时双帧 DMA 来不及 → 每拍 SKIP，raw 永旧。
         * 连续 SKIP 则 abort，下一拍重新 kick。
         */
        if (ctx->skip_busy_n < 0xFFu) {
            ctx->skip_busy_n++;
        }
        if (ctx->skip_busy_n >= 3u) {
            as5047_abort(e);
            ctx->skip_busy_n = 0u;
            encoder_profile_notify(e, ENC_EVT_ERROR, as5047_cyccnt(), 4U);
        } else {
            encoder_profile_notify(e, ENC_EVT_KICK_SKIP_BUSY, as5047_cyccnt(),
                                   (uint32_t)ctx->skip_busy_n);
        }
        return;
    }

    ctx->skip_busy_n = 0u;
    (void)as5047_kick_frame1(e);
}

static uint16_t as5047_chip_get_raw(const encoder_t *e)
{
    const as5047_ctx_t *ctx = (const as5047_ctx_t *)e->chip_ctx;

    if (ctx == NULL) {
        return 0U;
    }
    return ctx->raw;
}

static void as5047_chip_on_rx_complete(encoder_t *e)
{
    as5047_ctx_t *ctx = as5047_ctx(e);
    uint32_t t0;
    uint32_t t1;

    if (ctx == NULL || e->bus == NULL) {
        return;
    }

    if (ctx->phase == AS5047_PHASE_FRAME1) {
        t0 = as5047_cyccnt();

        encoder_spi_bus_cs_high(e->bus);
        encoder_spi_bus_cs_low(e->bus);
        if (encoder_spi_bus_restart_word(e->bus, &ctx->tx_nop, &ctx->rx_buf) != 0) {
            as5047_abort(e);
            encoder_profile_notify(e, ENC_EVT_ERROR, as5047_cyccnt(), 2U);
            return;
        }

        t1 = as5047_cyccnt();
        ctx->f1_cb_delta = t1 - t0;
        ctx->phase = AS5047_PHASE_FRAME2;
        encoder_profile_notify(e, ENC_EVT_F1_DONE, t1, ctx->f1_cb_delta);
        return;
    }

    if (ctx->phase == AS5047_PHASE_FRAME2) {
        t0 = as5047_cyccnt();

        encoder_spi_bus_cs_high(e->bus);
        ctx->raw = (uint16_t)(ctx->rx_buf & AS5047_RAW_MASK);
        ctx->phase = AS5047_PHASE_IDLE;

        t1 = as5047_cyccnt();
        encoder_profile_notify(e, ENC_EVT_F2_DONE, t1, t1 - t0);
    }
}

static void as5047_chip_on_error(encoder_t *e)
{
    as5047_abort(e);
    encoder_profile_notify(e, ENC_EVT_ERROR, as5047_cyccnt(), 3U);
}

static float as5047_chip_unwrap(encoder_t *e, uint16_t raw)
{
    return as5047_unwrap(as5047_ctx(e), raw);
}

const encoder_driver_t as5047_encoder_driver = {
    .init = as5047_chip_init,
    .async_init = as5047_chip_async_init,
    .kick = as5047_chip_kick,
    .get_raw = as5047_chip_get_raw,
    .on_rx_complete = as5047_chip_on_rx_complete,
    .on_error = as5047_chip_on_error,
    .unwrap = as5047_chip_unwrap,
};
