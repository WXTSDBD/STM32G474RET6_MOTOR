/**
 * @file time_port.h
 * @brief 单调时间 Port 契约（无 HAL）。P4 接 DWT / RTOS backend。
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
