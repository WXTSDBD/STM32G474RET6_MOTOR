/**
 * @file motor_trig_cfg.h
 * @date 2026-10-06
 * @brief 正余弦后端选择。1=LUT，2=CORDIC。

 *
 * 本头只有宏。改了要重编所有调用 trig 的文件。
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
 */

#ifndef MOTOR_TRIG_CFG_H
#define MOTOR_TRIG_CFG_H

/** 1=256 ? sin LUT + ????  2=?? CORDIC?Keil ? -DMOTOR_TRIG_BACKEND=2 ?? */
#define MOTOR_TRIG_BACKEND_LUT     1
#define MOTOR_TRIG_BACKEND_CORDIC  2

/*
 * 默认 LUT（bringup/math_tables）。无 LTO 时 sincos 在 motor_trig.h 内联。
 * 需要 CORDIC 对照时：-DMOTOR_TRIG_BACKEND=2。量产/移植保持 LUT。
 */
#ifndef MOTOR_TRIG_BACKEND
#define MOTOR_TRIG_BACKEND  MOTOR_TRIG_BACKEND_LUT
#endif
#if MOTOR_TRIG_BACKEND != MOTOR_TRIG_BACKEND_LUT && \
    MOTOR_TRIG_BACKEND != MOTOR_TRIG_BACKEND_CORDIC
#error "MOTOR_TRIG_BACKEND must be LUT(1) or CORDIC(2)"
#endif

#endif
