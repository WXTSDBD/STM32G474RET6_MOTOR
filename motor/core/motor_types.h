/**
 * @file motor_types.h
 * @date 2026-10-25
 * @brief 电机铭牌与控制器参数（"宏转表" 阶段 0 · 冻结接口）。
 *
 * 对应设计：docs/方案_宏转实验表_设计_2026-10-25.md §5.1。
 * 本阶段**只定义类型，不接任何逻辑**：没有文件 include 它，
 * 因此固件 hex 应与加文件前逐字相同。
 *
 * 用法（后续阶段）：
 *   config/plant/m1_n5065.c 里放一个 const motor_plant_t；
 *   config/routes/r_*.c 里放 const motor_gains_t；
 *   motor_route_bind() 把它们挂到 ctx 上，取代 M1_RS_OHM 之类的宏。
 */
#ifndef MOTOR_CORE_MOTOR_TYPES_H
#define MOTOR_CORE_MOTOR_TYPES_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 电机铭牌数据（一个电机一份）。 */
typedef struct {
    float rs_ohm;      /**< 相电阻，单位 Ω。 */
    float ld_h;        /**< d 轴电感，单位 H。 */
    float lq_h;        /**< q 轴电感，单位 H。 */
    float kt_nm_a;     /**< 力矩常数，单位 N·m/A。 */
    float psi_wb;      /**< 磁链，单位 Wb。 */
    float j_kgm2;      /**< 转动惯量，单位 kg·m²；不可辨识时写 0 并置 valid=0。 */
    float b_nms;       /**< 粘性阻尼，单位 N·m·s；不可辨识时写 0。 */
    uint8_t pole_pairs;/**< 极对数。 */
    uint8_t valid;     /**< 1=实测签收；0=手册/推断值。 */
    float vbus_v;      /**< 母线电压，单位 V。 */
    float deadtime_s;  /**< 功率级死区时间，单位 s。 */
} motor_plant_t;

/** 单个 PI 参数。 */
typedef struct {
    float fc_hz;    /**< 目标带宽，单位 Hz。 */
    float kp;       /**< 比例增益。 */
    float ki;       /**< 积分增益。 */
    float i_lim_a;  /**< 电流限幅，单位 A（速度/位置环填 0=不适用）。 */
    float v_lim_v;  /**< 电压限幅，单位 V（外环填 0=不适用）。 */
} pi_cfg_t;

/** 整套控制器参数（一个"档"一份）。 */
typedef struct {
    pi_cfg_t iq;        /**< q 轴电流环。 */
    pi_cfg_t id;        /**< d 轴电流环。 */
    pi_cfg_t spd;       /**< 速度环。 */
    pi_cfg_t pos;       /**< 位置环。 */
    float theta_lim;    /**< 角度误差保护门限，单位 rad。 */
} motor_gains_t;

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_CORE_MOTOR_TYPES_H */
