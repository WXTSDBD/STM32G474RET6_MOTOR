/**
 * @file motor_trig.c
 * @date 2026-10-06
 * @brief 按编译开关把正余弦分到 LUT 或 CORDIC。

 *
 * 节拍限制见 motor_trig.h 文件头。
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
 */

#include "motor_trig.h"
#include "motor_trig_cfg.h"
#include "motor_trig_backend.h"

void motor_trig_init(void)
{
#if MOTOR_TRIG_BACKEND == MOTOR_TRIG_BACKEND_CORDIC
    motor_trig_cordic_init();
#else
    motor_trig_lut_init();
#endif
}

/**
 * @brief 同时给出 cos 和 sin。指针不可为 NULL。
 */
void motor_trig_sincos(float rad, float *cos_out, float *sin_out)
{
#if MOTOR_TRIG_BACKEND == MOTOR_TRIG_BACKEND_CORDIC
    motor_trig_cordic_sincos(rad, cos_out, sin_out);
#else
    motor_trig_lut_sincos(rad, cos_out, sin_out);
#endif
}

/**
 * @brief 只要正弦。
 */
float motor_trig_sin(float rad)
{
    float c;
    float s;

    motor_trig_sincos(rad, &c, &s);
    return s;
}
