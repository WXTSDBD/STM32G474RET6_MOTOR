/**
 * @file deadband_module.h
 * @brief 死区补偿模块统一入口（论文 4.4 工程版）。
 *
 * 目录 `motor/deadband/` 集中全部死区相关实现，避免在 Drivers/foc 与 motor 根目录分散查找。
 *
 * | 文件 | 职责 |
 * |------|------|
 * | deadband.h / deadband.c | 运行时：FIXED / LUT、Ud 注入、abc duty 注入 |
 * | deadband_cal.h / deadband_cal.c | 标定：Pass0 capture、commit、双表 switch |
 * | deadband_geo.h / deadband_geo.c | 论文 4.4 电压域反变换与 geo phase 建表 |
 * | deadband_id_cal.h / .c        | Id 锁轴扫表状态机编排 |
 * | deadband_flow.h / .c          | 配方表：Id 标定 → ident Bode/阶跃 |
 * | deadband_service.h / .c       | Service 门面：boot / profile / Ud 注入 |
 *
 * 调用方只需 `#include "deadband_module.h"`，或按需单独包含 deadband.h / deadband_cal.h。
 *
 * 配置宏见 config/motor_params_m1.h（M1_DEADBAND_*、M1_ID_CAL_*）。
 * 注入点：motor/foc_svpwm.c（duty abc）、deadband_id_cal（标定编排）。
 */

#ifndef DEADBAND_MODULE_H
#define DEADBAND_MODULE_H

#include "deadband.h"
#include "deadband_cal.h"
#include "deadband_geo.h"
#include "deadband_lut_baked_m1.h"
#include "deadband_service.h"
#include "deadband_id_cal.h"
#include "deadband_flow.h"

#endif /* DEADBAND_MODULE_H */
