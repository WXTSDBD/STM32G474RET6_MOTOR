/**
 * @file motor_bind.c
 * @date 2026-10-25
 * @brief route 绑定骨架（"宏转表" 阶段 0 · 冻结接口）。
 *
 * 对应设计：docs/方案_宏转实验表_设计_2026-10-25.md §4/§6 阶段 0。
 * 阶段 0 要求：**不改现有 .c、hex 逐字相同**。
 * 因此这里只做指针保存（无调用者，链接器会把它当死代码移除），
 * 后续阶段再把 motor_current/outer 的读取点切到 s_route 上。
 */
#include "core/motor_route.h"

#include <stddef.h>

/** 当前生效的 route（阶段 0 只写不读）。 */
static const motor_route_t *s_route;

/**
 * @brief 绑定 route（启动一次）。
 * @param ctx   电机上下文（阶段 0 未使用）。
 * @param route 目标 route；NULL 表示解绑。
 */
void motor_route_bind(motor_context_t *ctx, const motor_route_t *route)
{
    (void)ctx;
    s_route = route;
}

/**
 * @brief 取当前 route（供后续阶段读取）。
 * @return 当前 route；未绑定时为 NULL。
 */
const motor_route_t *motor_route_active(void)
{
    return s_route;
}
