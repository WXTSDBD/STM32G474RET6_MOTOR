#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

#include "encoder_spi_bus.h"

typedef struct encoder encoder_t;

typedef enum {
    ENC_EVT_KICK_DONE = 0,
    ENC_EVT_F1_DONE,
    ENC_EVT_F2_DONE,
    ENC_EVT_KICK_SKIP_BUSY,
    ENC_EVT_ERROR,
} enc_event_t;

typedef struct {
    enc_event_t ev;
    uint32_t cyccnt;
    uint32_t aux;
} enc_profile_event_t;

typedef void (*encoder_profile_cb_t)(const encoder_t *e, const enc_profile_event_t *ev);

typedef struct {
    int (*init)(encoder_t *e);
    int (*async_init)(encoder_t *e);
    void (*kick)(encoder_t *e);
    uint16_t (*get_raw)(const encoder_t *e);
    void (*on_rx_complete)(encoder_t *e);
    void (*on_error)(encoder_t *e);
    float (*unwrap)(encoder_t *e, uint16_t raw);
} encoder_driver_t;

struct encoder {
    const encoder_driver_t *drv;
    void *chip_ctx;
    encoder_spi_bus_t *bus;
    encoder_profile_cb_t profile_cb;
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

/** Single-turn electrical angle [0, 2pi) rad from raw; for SVPWM @ 20kHz. */
float encoder_get_theta_el(const encoder_t *e, uint16_t raw, uint8_t pole_pairs, float offset_rad);

void encoder_set_theta_el_offset(encoder_t *e, float offset_rad);
float encoder_get_theta_el_offset(const encoder_t *e);

void encoder_profile_notify(const encoder_t *e, enc_event_t ev, uint32_t cyccnt, uint32_t aux);

#endif
