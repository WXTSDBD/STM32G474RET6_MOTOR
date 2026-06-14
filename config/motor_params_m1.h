/**
 * @file motor_params_m1.h
 * @brief M1 电机硬件常数：电流采样链路的标度（初始化预计算，热路径只乘不除）。
 *
 * ADC2 三相：外部放大 ×10，采样电阻 10 mΩ，12bit 单端 @ VDDA。
 */

#ifndef MOTOR_PARAMS_M1_H
#define MOTOR_PARAMS_M1_H

/** VDDA / ADC 参考（V） */
#define M1_ADC_VREF_V       3.3f

/** 相电流采样电阻（Ω），10 mΩ */
#define M1_ADC_SHUNT_OHM    0.01f

/** 外部电流放大倍数 */
#define M1_ADC_AMP_GAIN     10.0f

/** 安培/LSB：Vref / (4096 × R_shunt × Gain)，热路径仅做 (raw-offset)*scale */
#define M1_ADC_SCALE_A_LSB  (M1_ADC_VREF_V / (4096.0f * M1_ADC_SHUNT_OHM * M1_ADC_AMP_GAIN))

#endif
