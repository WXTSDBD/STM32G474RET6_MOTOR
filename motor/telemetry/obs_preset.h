/**
 * @file obs_preset.h
 * @date 2026-10-31
 * @brief 旧 telem layout_id → 名字预设，经 obs_session_set 生效（仅 RTOS）。
 */
#ifndef MOTOR_TELEMETRY_OBS_PRESET_H
#define MOTOR_TELEMETRY_OBS_PRESET_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 与 TELEM_LAYOUT_* 同号，避免配方改两处。 */
#define OBS_PRESET_FOC_DQ      0u
#define OBS_PRESET_SIGNOFF     1u
#define OBS_PRESET_SPEED       2u
#define OBS_PRESET_IDENT_DUTY  3u
#define OBS_PRESET_OPT         4u

uint8_t obs_preset_default_id(void);
uint8_t obs_preset_apply(uint8_t id, uint8_t decim);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_TELEMETRY_OBS_PRESET_H */
