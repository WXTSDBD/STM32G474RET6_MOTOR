/**
 * @file motor_trig.h
 * @date 2026-10-06
 * @brief 电角正余弦。后端是 LUT 或 CORDIC。

 *
 * sincos 只允许从电流环节拍调用。
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
 */

#ifndef MOTOR_TRIG_H
#define MOTOR_TRIG_H

#ifdef __cplusplus
extern "C" {
#endif

void motor_trig_init(void);
void motor_trig_sincos(float rad, float *cos_out, float *sin_out);
float motor_trig_sin(float rad);

#ifdef __cplusplus
}
#endif

#endif
