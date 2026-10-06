/**
 * @file motor_trig_backend.h
 * @date 2026-10-06
 * @brief LUT 与 CORDIC 后端原型。

 *
 * 调用方走 motor_trig.h，不要直接点名后端。
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
 */

#ifndef MOTOR_TRIG_BACKEND_H
#define MOTOR_TRIG_BACKEND_H

void motor_trig_cordic_init(void);
void motor_trig_cordic_sincos(float rad, float *cos_out, float *sin_out);

void motor_trig_lut_init(void);
void motor_trig_lut_sincos(float rad, float *cos_out, float *sin_out);

#endif
