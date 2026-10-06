/**
 * @file deadband.h
 * @date 2026-10-06
 * @brief 逆变器死区补偿：固定符号法或查表。

 *
 * apply_duty 只允许从电流环节拍、写出 PWM 之前调用。
 * 任务只改 mode 和 LUT，不要在任务里改占空比。
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
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

#define M1_DEADBAND_LUT_MAX 160u

typedef struct {
    m1_deadband_mode_t mode;
    /** 固定模式：每相补偿电压幅值（V） */
    float v_comp_v;
    /** |i| 低于此值不补偿，避免过零 sign 抖动 */
    float i_zero_a;
    /** LUT 模式：电流幅值表（A），单调递增；单表或 phase A */
    const float *lut_amps;
    /** LUT 模式：对应误差电压（V，恒为正幅值） */
    const float *lut_vals;
    uint8_t lut_len;
    /** 1=abc 各相查 f_a/f_b/f_c 三表 */
    uint8_t lut_triplet;
    const float *lut_amps_ph[3];
    const float *lut_vals_ph[3];
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
 * @brief 注册分相 abc LUT（A/B/C 各 len 点）；runtime apply_duty 各查各表。
 */
void deadband_set_phase_luts(const float *amps_a, const float *vals_a,
                             const float *amps_b, const float *vals_b,
                             const float *amps_c, const float *vals_c,
                             uint8_t len);

/** d 轴域（1）或 phase 域（0）；影响 LOW_FLAT 电流边界 */
void deadband_set_lut_domain(uint8_t is_d_domain);

/** Pass1=1（Ud）；Iq/日常=0（abc duty） */
void deadband_set_runtime_apply_ud(uint8_t apply_ud);

/** commit 后注册 d 表供 RUNTIME_GEO 查 udinv(|i|) */
void deadband_set_geo_dlut(const float *amps, const float *vals, uint8_t len);

/** TWO_CLUSTER：cluster 0=30° 族，1=0° 族 plut */
void deadband_set_cluster_luts(const float *amps_a, const float *vals_a,
                               const float *amps_b, const float *vals_b,
                               uint8_t len);

/** phase 表 apply_duty 运行缩放（NVM 加载 / commit 后覆盖编译期默认） */
void deadband_set_lut_runtime_scale(float scale);
float deadband_get_lut_runtime_scale(void);

/**
 * @brief 单相补偿电压（V，带符号）；OFF 模式返回 0。FIXED/LUT abc 用。
 */
float deadband_comp_v(float i_a);

/** phase 0=A, 1=B, 2=C；单表时与 deadband_comp_v 相同 */
float deadband_comp_v_ph(uint8_t phase, float i_a);

/**
 * @brief d 轴 LUT 补偿（V，带 Id 符号）；仅 LUT+APPLY_UD=1 时在 Ud 上叠加。
 */
float deadband_ud_comp_v(float id_a);

/**
 * @brief 在三相归一化占空比 [0,1] 上叠加补偿并 clamp。
 * @param theta_el  电角 (rad)；RUNTIME_GEO / TWO_CLUSTER 使用
 * @param id_dq     d 轴电流 (A)；RUNTIME_GEO 查表幅值用 hypot(Id,Iq)
 * @param iq_dq     q 轴电流 (A)
 */
void deadband_apply_duty(float ia, float ib, float ic,
                         float theta_el, float id_dq, float iq_dq,
                         float *duty_a, float *duty_b, float *duty_c);

#ifdef __cplusplus
}
#endif

#endif /* DEADBAND_H */
