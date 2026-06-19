/**
 * @file deadband.h
 * @brief 逆变器死区补偿：固定 V_comp（符号法）+ LUT 查表接口（Step 3 后续）。
 *
 * 补偿施加在 SVPWM 归一化占空比 Ta/Tb/Tc 上（方案 A）。
 */

#ifndef DEADBAND_H
#define DEADBAND_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    M1_DEADBAND_MODE_OFF = 0,
    M1_DEADBAND_MODE_FIXED,
    M1_DEADBAND_MODE_LUT,
} m1_deadband_mode_t;

#define M1_DEADBAND_LUT_MAX 32u

typedef struct {
    m1_deadband_mode_t mode;
    /** 固定模式：每相补偿电压幅值（V） */
    float v_comp_v;
    /** |i| 低于此值不补偿，避免过零 sign 抖动 */
    float i_zero_a;
    /** LUT 模式：电流幅值表（A），单调递增 */
    const float *lut_amps;
    /** LUT 模式：对应误差电压（V，恒为正幅值） */
    const float *lut_vals;
    uint8_t lut_len;
} m1_deadband_cfg_t;

/** 按 motor_params_m1.h 初始化默认固定补偿（591 ns）。 */
void deadband_init(void);

void deadband_set_mode(m1_deadband_mode_t mode);
m1_deadband_mode_t deadband_get_mode(void);
const m1_deadband_cfg_t *deadband_get_cfg(void);

/**
 * @brief 注册 LUT（Step 3 扫表后调用）；len 须 >= 2。
 */
void deadband_set_lut(const float *amps, const float *vals, uint8_t len);

/**
 * @brief 单相补偿电压（V，带符号）；OFF 模式返回 0。
 */
float deadband_comp_v(float i_a);

/**
 * @brief 在三相归一化占空比 [0,1] 上叠加补偿并 clamp。
 */
void deadband_apply_duty(float ia, float ib, float ic,
                         float *duty_a, float *duty_b, float *duty_c);

#ifdef __cplusplus
}
#endif

#endif
