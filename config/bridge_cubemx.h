/**
 * @file bridge_cubemx.h
 * @date 2026-10-06
 * @brief CubeMX HAL 句柄到轴配置的唯一桥。

 *
 * 驱动层不要直接引用 hadc2/htim8。
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
 */

#ifndef BRIDGE_CUBEMX_H
#define BRIDGE_CUBEMX_H

#include "bsp_axes.h"

#ifdef __cplusplus
extern "C" {
#endif

void bridge_cubemx_apply_axis(bsp_axis_id_t id, bsp_axis_t *axis);

#ifdef __cplusplus
}
#endif

#endif
