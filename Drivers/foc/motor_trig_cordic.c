/**
 * @file motor_trig_cordic.c
 * @date 2026-10-06
 * @brief 片上 CORDIC 求正余弦。

 *
 * 初始化一次。sincos 在电流环节拍里等结果。
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
 */

#include "motor_trig_cfg.h"

#if MOTOR_TRIG_BACKEND == MOTOR_TRIG_BACKEND_CORDIC

#include "motor_trig_backend.h"
#include "cordic.h"
#include "stm32g4xx_hal.h"
#include "stm32g474xx.h"

#define MOTOR_TRIG_Q31_SCALE  2147483648.0f
#define MOTOR_TRIG_INV_PI     0.31830988618f
#define MOTOR_TRIG_PI         3.14159265359f
#define MOTOR_TRIG_TWO_PI     6.28318530718f

extern CORDIC_HandleTypeDef hcordic;

static uint8_t s_cordic_ready;

/** CORDIC Cosine: WDATA = angle/π in Q1.31, valid only for angle ∈ [-π, +π]. */
static float motor_trig_wrap_pm_pi(float rad)
{
    while (rad > MOTOR_TRIG_PI) {
        rad -= MOTOR_TRIG_TWO_PI;
    }
    while (rad <= -MOTOR_TRIG_PI) {
        rad += MOTOR_TRIG_TWO_PI;
    }
    return rad;
}

static float motor_trig_q31_to_float(int32_t q31)
{
    return (float)q31 / MOTOR_TRIG_Q31_SCALE;
}

static int32_t motor_trig_rad_to_q31(float rad)
{
    rad = motor_trig_wrap_pm_pi(rad);
    return (int32_t)(rad * (MOTOR_TRIG_Q31_SCALE * MOTOR_TRIG_INV_PI));
}

void motor_trig_cordic_init(void)
{
    CORDIC_ConfigTypeDef cfg = {0};

    cfg.Function = CORDIC_FUNCTION_COSINE;
    cfg.Precision = CORDIC_PRECISION_6CYCLES;
    cfg.Scale = CORDIC_SCALE_0;
    cfg.NbWrite = CORDIC_NBWRITE_1;
    cfg.NbRead = CORDIC_NBREAD_2;
    cfg.InSize = CORDIC_INSIZE_32BITS;
    cfg.OutSize = CORDIC_OUTSIZE_32BITS;

    (void)HAL_CORDIC_Configure(&hcordic, &cfg);
    s_cordic_ready = 1u;
}

void motor_trig_cordic_sincos(float rad, float *cos_out, float *sin_out)
{
    int32_t z;
    int32_t cos_q31;
    int32_t sin_q31;

    if (cos_out == NULL || sin_out == NULL) {
        return;
    }

    if (s_cordic_ready == 0u) {
        motor_trig_cordic_init();
    }

    z = motor_trig_rad_to_q31(rad);
    CORDIC->WDATA = (uint32_t)z;

    while ((CORDIC->CSR & CORDIC_CSR_RRDY) == 0U) {
    }

    cos_q31 = (int32_t)CORDIC->RDATA;

    while ((CORDIC->CSR & CORDIC_CSR_RRDY) == 0U) {
    }

    sin_q31 = (int32_t)CORDIC->RDATA;

    *cos_out = motor_trig_q31_to_float(cos_q31);
    *sin_out = motor_trig_q31_to_float(sin_q31);
}

#endif
