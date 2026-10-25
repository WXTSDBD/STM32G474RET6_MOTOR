/**
 * @file exp_rev141_params.h
 * @date 2026-10-25
 * @brief ±2000 金样实验参数（原 `M1_HFI_REV141_*` 宏，阶段 2 表化）。
 *
 * 数据放在 .c 里（`config/exp_rev141_params.c`）。
 * 正反转次数和目标转速写在段表里。hfi_sqwave 只读 enc_rpm_lp_a。
 */
#ifndef CONFIG_EXP_REV141_PARAMS_H
#define CONFIG_EXP_REV141_PARAMS_H

#ifdef __cplusplus
extern "C" {
#endif

/** ±2000 实验的 7 个参数。 */
typedef struct {
    unsigned pairs;          /**< 无感正反对的轮数。 */
    unsigned sensed_pairs;   /**< 无感跑完后跟几对有感。 */
    float target_rpm;        /**< 正/反目标转速幅值，rpm。 */
    float stop_rpm;          /**< |n| 低于此值算刹到 0，rpm。 */
    float stop_hold_s;       /**< 刹到 0 后短停时间，s。 */
    float brake_timeout_s;   /**< 刹停段最长等待，s。 */
    float enc_rpm_lp_a;      /**< enc 转速门限低通系数。 */
} exp_rev141_params_t;

/** 当前生效参数（const 数据，编译期初始化）。 */
extern const exp_rev141_params_t g_exp_rev141_params;

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_EXP_REV141_PARAMS_H */
