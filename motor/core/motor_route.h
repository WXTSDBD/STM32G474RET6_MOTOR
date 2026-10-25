/**
 * @file motor_route.h
 * @date 2026-10-25
 * @brief route：一台机器"怎么跑"（"宏转表" 阶段 0 · 冻结接口）。
 *
 * 对应设计：docs/方案_宏转实验表_设计_2026-10-25.md §5.2。
 * route = plant + gains + 五张 ops 表指针 + atrb（外环模式/初始指令）。
 *
 * 本阶段只定义类型 + 声明 bind 函数，不接逻辑（hex 不变）。
 * 后续阶段：config/routes/r_*.c 各定义一个 const motor_route_t，
 * 启动时 boot 只选 route_id，不再改 profile 宏。
 */
#ifndef MOTOR_CORE_MOTOR_ROUTE_H
#define MOTOR_CORE_MOTOR_ROUTE_H

#include <stdint.h>

#include "motor_types.h"
#include "motor_ops.h"
#include "../motor_context.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 外环模式（与现有 M1_OUTER_* 一一对应，迁移期并存）。 */
#define MOTOR_OUTER_DISABLED 0u
#define MOTOR_OUTER_TORQUE   1u
#define MOTOR_OUTER_SPEED    2u
#define MOTOR_OUTER_POSITION 3u
#define MOTOR_OUTER_MIT      4u

/** 一条 route：把"这台机器用什么参数、走哪套算法"打包。 */
typedef struct {
    const char *name;            /**< 例如 "m1_n5065_sensored_speed"。 */
    uint8_t route_id;            /**< 启动配置里选它用。 */
    const motor_plant_t *plant;  /**< 铭牌。 */
    const motor_gains_t *gains;  /**< 控制器参数。 */
    const angle_ops_t *angle;    /**< 角度源（必选）。 */
    const outer_ops_t *outer;    /**< 外环（必选）。 */
    const ff_ops_t *ff;          /**< 前馈链（可为空）。 */
    const iref_ops_t *iref;      /**< 电流给定（可为空=纯 d 轴 0）。 */
    const telem_ops_t *telem;    /**< 遥测布局（必选）。 */
    uint8_t outer_mode;          /**< 上电默认外环模式（MOTOR_OUTER_*）。 */
    uint8_t exp_id;              /**< 上电默认实验 id（0=不跑实验）。 */
    uint8_t reserved[2];
} motor_route_t;

/**
 * @brief 把 route 挂到上下文（启动一次，之后只读）。
 * @param ctx   电机上下文。
 * @param route 目标 route；NULL=解绑。
 */
void motor_route_bind(motor_context_t *ctx, const motor_route_t *route);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_CORE_MOTOR_ROUTE_H */
