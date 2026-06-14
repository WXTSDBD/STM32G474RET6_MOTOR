/**
 * @file bridge_cubemx.h
 * @brief CubeMX 生成 HAL 句柄 → bsp_axis 配置的唯一桥接头。
 *
 * 除本文件对应 .c 外，驱动层与 adc_sample 不得直接引用 hadc2/htim8 等符号。
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
