/**
 * @file time_port.h
 * @date 2026-10-06
 * @brief 单调时间：毫秒和 DWT 周期。

 *
 * init 在上电调用一次。cycles 可在中断里读。
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
 */

#ifndef TIME_PORT_H
#define TIME_PORT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

uint32_t time_port_ms(void);
uint32_t time_port_cycles(void);
void time_port_init(uint32_t cpu_mhz);

#ifdef __cplusplus
}
#endif

#endif
