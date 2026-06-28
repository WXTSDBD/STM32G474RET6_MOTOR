#include "encoder.h"

#include <stddef.h>

#include "as5047.h"

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

void encoder_set_profile_cb(encoder_t *e, encoder_profile_cb_t cb)
{
    e->profile_cb = cb;
}

int encoder_async_init(encoder_t *e)
{
    if (e == NULL || e->drv == NULL || e->drv->async_init == NULL) {
        return -1;
    }
    return e->drv->async_init(e);
}

void encoder_kick(encoder_t *e)
{
    if (e != NULL && e->drv != NULL && e->drv->kick != NULL) {
        e->drv->kick(e);
    }
}

uint16_t encoder_get_raw(const encoder_t *e)
{
    if (e == NULL || e->drv == NULL || e->drv->get_raw == NULL) {
        return 0U;
    }
    return e->drv->get_raw(e);
}

void encoder_on_spi_rx_complete(encoder_t *e)
{
    if (e != NULL && e->drv != NULL && e->drv->on_rx_complete != NULL) {
        e->drv->on_rx_complete(e);
    }
}

void encoder_on_spi_error(encoder_t *e)
{
    if (e != NULL && e->drv != NULL && e->drv->on_error != NULL) {
        e->drv->on_error(e);
    }
}

float encoder_get_angle(encoder_t *e, uint16_t raw)
{
    if (e == NULL || e->drv == NULL || e->drv->unwrap == NULL) {
        return 0.0f;
    }
    return e->drv->unwrap(e, raw);
}

float encoder_get_theta_el(const encoder_t *e, uint16_t raw, uint8_t pole_pairs, float offset_rad)
{
    (void)e;
    return as5047_raw_to_theta_el(raw, pole_pairs, offset_rad);
}

void encoder_set_theta_el_offset(encoder_t *e, float offset_rad)
{
    if (e != NULL) {
        e->theta_el_offset_rad = offset_rad;
    }
}

float encoder_get_theta_el_offset(const encoder_t *e)
{
    if (e == NULL) {
        return 0.0f;
    }
    return e->theta_el_offset_rad;
}

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
