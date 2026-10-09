/**
 * @file encoder_cal.c
 * @date 2026-10-06
 * @brief 编码器零偏锁转子标定实现。
 *
 * 只认 encoder_t*：阻塞读角与等 idle 走驱动钩子，不绑具体芯片类型。
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
 */

#include "encoder_cal.h"

#include <stddef.h>

#include "foc_svpwm.h"
#include "time_port.h"

#define ENCODER_CAL_TWO_PI 6.28318530718f
#define ENCODER_CAL_DMA_WAIT_MS 2u

/** 1=锁转子标定中，JEOC 走 encoder_cal_jeoc_tick。 */
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

/**
 * @brief 把电角折进 [0, 2π)。
 * @param angle 电角，单位 rad。
 * @return 折回后的电角，单位 rad。
 */
float encoder_cal_wrap_2pi(float angle)
{
    return encoder_cal_wrap_2pi_local(angle);
}

/**
 * @brief 按宏决定是否从零偏里再减 π。
 * @param add_raw 原始附加偏置，单位 rad。
 * @return 处理后的偏置，单位 rad。
 */
float encoder_cal_apply_pi_offset(float add_raw)
{
#if ENCODER_CAL_APPLY_PI_OFFSET
    return encoder_cal_wrap_2pi_local(add_raw - 3.14159265f);
#else
    return encoder_cal_wrap_2pi_local(add_raw);
#endif
}

/**
 * @brief 标定期间每拍用固定 θ_ref 和 Ud 吸转子。
 * @note 必须用 Ud、Uq=0。若 Uq 且 Ud 恰为 0，SVPWM 会走 angle+π/2
 *       捷径，与闭环 Park（ud 很少恰为 0）差 90°，表现为 iq 顶满转子钉死。
 * @param axis 轴实例。不可为 NULL。
 */
void encoder_cal_jeoc_tick(bsp_axis_t *axis)
{
    if (axis == NULL) {
        return;
    }
    /* foc_svpwm_apply(Uq, Ud, θ)：锁 d 轴 → Uq=0，Ud=s_uq_lock（电压幅值复用宏）。 */
    foc_svpwm_apply(axis, 0.0f, s_uq_lock, s_theta_ref);
}

/**
 * @brief 锁转子后阻塞读角，写出电角零偏。
 * @param axis 轴实例，不可为 NULL。
 * @param enc 编码器，不可为 NULL。
 * @param pole_pairs 极对数，不可为 0。
 * @param uq_lock 锁转子 Uq，单位 V。
 * @param settle_ms 等待稳定的毫秒数。
 * @param sample_count 平均采样次数，不可为 0。
 * @param add_out 写出电角附加偏置，单位 rad。不可为 NULL。
 * @return 成功为 true。
 * @note 本函数会空转等待，只能在任务里调用。
 */
bool encoder_cal_run_lock(bsp_axis_t *axis,
                          encoder_t *enc,
                          uint8_t pole_pairs,
                          float uq_lock,
                          uint32_t settle_ms,
                          uint16_t sample_count,
                          float *add_out)
{
    uint32_t t0;
    uint32_t raw_sum;
    uint16_t raw_avg;
    uint16_t i;
    float theta_meas;

    if (axis == NULL || axis->pwm == NULL || enc == NULL || add_out == NULL ||
        pole_pairs == 0U || sample_count == 0U) {
        return false;
    }

    s_uq_lock = uq_lock;
    s_theta_ref = 0.0f;

    g_encoder_cal_active = 1U;
    t0 = time_port_ms();
    while ((time_port_ms() - t0) < settle_ms) {
        /* JEOC 内 encoder_cal_jeoc_tick 刷新 SVPWM */
    }
    g_encoder_cal_active = 0U;

    raw_sum = 0U;
    for (i = 0U; i < sample_count; i++) {
        raw_sum += (uint32_t)encoder_blocking_read_angle(enc);
    }
    raw_avg = (uint16_t)(raw_sum / (uint32_t)sample_count);

    theta_meas = encoder_get_theta_el(enc, raw_avg, pole_pairs, 0.0f);
    *add_out = encoder_cal_wrap_2pi_local(-theta_meas);

    encoder_kick(enc);

    t0 = time_port_ms();
    while (encoder_xfer_busy(enc) != 0U) {
        if ((time_port_ms() - t0) >= ENCODER_CAL_DMA_WAIT_MS) {
            break;
        }
    }

    return true;
}
