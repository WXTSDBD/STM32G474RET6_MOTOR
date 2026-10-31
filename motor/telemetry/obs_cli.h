/**
 * @file obs_cli.h
 * @date 2026-10-31
 * @brief OBS 控制口（RX DMA + RTOS 换通道）。
 */
#ifndef MOTOR_TELEMETRY_OBS_CLI_H
#define MOTOR_TELEMETRY_OBS_CLI_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

uint8_t obs_cli_telem_enabled(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_TELEMETRY_OBS_CLI_H */
