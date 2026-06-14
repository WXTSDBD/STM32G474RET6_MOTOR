/**
 * @file encoder_cal.c
 * @brief 编码器零偏锁转子标定实现。
 */

#include "encoder_cal.h"

#include "as5047.h"
#include "main.h"
#include "trans.h"

#define ENCODER_CAL_TWO_PI 6.28318530718f
#define ENCODER_CAL_DMA_WAIT_MS 2u

volatile uint8_t g_encoder_cal_active;

static float s_uq_lock = ENCODER_CAL_UQ_LOCK;
static float s_theta_ref;

static float encoder_cal_wrap_2pi_local(float angle)
{
    while (angle >= ENCODER_CAL_TWO_PI) {
        angle -= ENCODER_CAL_TWO_PI;
    }
    while (angle < 0.0f) {
        angle += ENCODER_CAL_TWO_PI;
    }
    return angle;
}

float encoder_cal_wrap_2pi(float angle)
{
    return encoder_cal_wrap_2pi_local(angle);
}

float encoder_cal_apply_pi_offset(float add_raw)
{
#if ENCODER_CAL_APPLY_PI_OFFSET
    return encoder_cal_wrap_2pi_local(add_raw - 3.14159265f);
#else
    return encoder_cal_wrap_2pi_local(add_raw);
#endif
}

static as5047_ctx_t *encoder_cal_as5047_ctx(const encoder_t *enc)
{
    if (enc == NULL) {
        return NULL;
    }
    return (as5047_ctx_t *)enc->chip_ctx;
}

void encoder_cal_jeoc_tick(TIM_HandleTypeDef *htim)
{
    if (htim == NULL) {
        return;
    }
    setPhaseVoltage(htim, s_uq_lock, 0.0f, s_theta_ref);
}

bool encoder_cal_run_lock(encoder_t *enc,
                          TIM_HandleTypeDef *htim,
                          uint8_t pole_pairs,
                          float uq_lock,
                          uint32_t settle_ms,
                          uint16_t sample_count,
                          float *add_out)
{
    as5047_ctx_t *ctx;
    uint32_t t0;
    uint32_t raw_sum;
    uint16_t raw_avg;
    uint16_t i;
    float theta_meas;

    if (enc == NULL || htim == NULL || add_out == NULL || pole_pairs == 0U || sample_count == 0U) {
        return false;
    }

    ctx = encoder_cal_as5047_ctx(enc);
    if (ctx == NULL || ctx->hal == NULL) {
        return false;
    }

    s_uq_lock = uq_lock;
    s_theta_ref = 0.0f;

    g_encoder_cal_active = 1U;
    t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < settle_ms) {
        /* JEOC 内 encoder_cal_jeoc_tick 刷新 SVPWM */
    }
    g_encoder_cal_active = 0U;

    raw_sum = 0U;
    for (i = 0U; i < sample_count; i++) {
        raw_sum += (uint32_t)(as5047_blocking_read(ctx->hal, AS5047_ANGLEUNC) & AS5047_RAW_MASK);
    }
    raw_avg = (uint16_t)(raw_sum / (uint32_t)sample_count);

    theta_meas = encoder_get_theta_el(enc, raw_avg, pole_pairs, 0.0f);
    *add_out = encoder_cal_wrap_2pi_local(-theta_meas);

    encoder_kick(enc);

    t0 = HAL_GetTick();
    while (ctx->phase != (uint8_t)AS5047_PHASE_IDLE) {
        if ((HAL_GetTick() - t0) >= ENCODER_CAL_DMA_WAIT_MS) {
            break;
        }
    }

    return true;
}
