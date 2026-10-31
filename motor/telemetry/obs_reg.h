/**
 * @file obs_reg.h
 * @date 2026-10-31
 * @brief 观测登记 + 会话。换表只在 RTOS；ISR 只按当前表拷贝。
 */
#ifndef MOTOR_TELEMETRY_OBS_REG_H
#define MOTOR_TELEMETRY_OBS_REG_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define OBS_F32 0u
#define OBS_CH_MAX 12u

typedef struct {
    const char *name;
    void *addr;
    uint16_t size;
    uint8_t type;
} obs_entry_t;

#define OBSERVE_AS(sym, nm, v)                                                 \
    const obs_entry_t obs_##sym __attribute__((used, section(".obs_tab"))) =   \
        { (nm), (void *)&(v), (uint16_t)sizeof(v), OBS_F32 }

void obs_reg_init(void);
uint8_t obs_count(void);
const obs_entry_t *obs_find(const char *name);

/** RTOS 调用：立刻换会话。0=ok 1=未知名 2=带宽 3=参数。 */
uint8_t obs_session_set(const char *const names[], uint8_t n, uint8_t decim);
uint8_t obs_session_n(void);
uint8_t obs_session_decim(void);
uint32_t obs_session_seq(void);
uint8_t obs_session_active(void);

/** ISR/遥测：只读当前表，不换表。 */
void obs_sample(float vals[OBS_CH_MAX]);

void obs_drop_add(uint32_t n);
uint32_t obs_drop_get(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_TELEMETRY_OBS_REG_H */
