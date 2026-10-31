/**
 * @file obs_reg.c
 * @date 2026-10-31
 * @brief 登记表查找；会话换表仅 RTOS；ISR 只 sample。
 */
#include "telemetry/obs_reg.h"

#include <string.h>

#include "cmsis_compiler.h"

extern const obs_entry_t Image$$ER_OBS_TAB$$Base;
extern const obs_entry_t Image$$ER_OBS_TAB$$Limit;

typedef struct {
    const volatile float *addr[OBS_CH_MAX];
    uint8_t n;
    uint8_t decim;
    uint32_t seq;
} obs_slot_t;

static obs_slot_t s_slot[2];
static volatile uint8_t s_active;
static uint32_t s_drop;

void obs_reg_init(void)
{
    memset(&s_slot[0], 0, sizeof(s_slot));
    s_slot[0].decim = 2u;
    s_slot[1].decim = 2u;
    s_active = 0u;
    s_drop = 0u;
}

uint8_t obs_count(void)
{
    const obs_entry_t *b = &Image$$ER_OBS_TAB$$Base;
    const obs_entry_t *e = &Image$$ER_OBS_TAB$$Limit;
    uint32_t bytes;

    if (e < b) {
        return 0u;
    }
    bytes = (uint32_t)((const uint8_t *)e - (const uint8_t *)b);
    return (uint8_t)(bytes / sizeof(obs_entry_t));
}

const obs_entry_t *obs_entry_at(uint8_t i)
{
    if (i >= obs_count()) {
        return 0;
    }
    return &Image$$ER_OBS_TAB$$Base + i;
}

const obs_entry_t *obs_find(const char *name)
{
    uint8_t i;
    uint8_t n;

    if (name == 0) {
        return 0;
    }
    n = obs_count();
    for (i = 0u; i < n; i++) {
        const obs_entry_t *e = obs_entry_at(i);
        if ((e != 0) && (e->name != 0) && (strcmp(e->name, name) == 0)) {
            return e;
        }
    }
    return 0;
}

static uint8_t obs_bw_ok(uint8_t n, uint8_t decim)
{
    uint32_t rate;

    if ((n == 0u) || (n > OBS_CH_MAX) || (decim == 0u)) {
        return 0u;
    }
    rate = (20000u / (uint32_t)decim) * (uint32_t)n;
    return (rate <= 80000u) ? 1u : 0u;
}

/**
 * @brief RTOS 换表：写满非活动槽后原子切 active（ISR 不参与切换）。
 */
uint8_t obs_session_set(const char *const names[], uint8_t n, uint8_t decim)
{
    uint8_t i;
    uint8_t nxt;
    obs_slot_t *dst;

    if ((names == 0) || (n == 0u) || (n > OBS_CH_MAX) || (decim == 0u)) {
        return 3u;
    }
    if (obs_bw_ok(n, decim) == 0u) {
        return 2u;
    }

    nxt = (uint8_t)(s_active ^ 1u);
    dst = &s_slot[nxt];
    memset(dst, 0, sizeof(*dst));
    dst->n = n;
    dst->decim = decim;
    dst->seq = 0u;

    for (i = 0u; i < n; i++) {
        const obs_entry_t *e = obs_find(names[i]);
        if (e == 0) {
            return 1u;
        }
        dst->addr[i] = (const volatile float *)e->addr;
    }

    __DMB();
    s_active = nxt; /* 仅 RTOS 写 */
    __DMB();
    return 0u;
}

uint8_t obs_session_n(void)
{
    return s_slot[s_active].n;
}

uint8_t obs_session_decim(void)
{
    return s_slot[s_active].decim;
}

uint32_t obs_session_seq(void)
{
    return s_slot[s_active].seq;
}

uint8_t obs_session_active(void)
{
    return (s_slot[s_active].n > 0u) ? 1u : 0u;
}

/**
 * @brief 遥测拍：只解引用当前表。
 */
void obs_sample(float vals[OBS_CH_MAX])
{
    uint8_t i;
    obs_slot_t *s;

    if (vals == 0) {
        return;
    }
    s = &s_slot[s_active];
    for (i = 0u; i < OBS_CH_MAX; i++) {
        vals[i] = (i < s->n) ? *(s->addr[i]) : 0.0f;
    }
    s->seq++;
    if (s->seq >= 16777216u) {
        s->seq = 0u;
    }
}

uint32_t obs_drop_get(void)
{
    return s_drop;
}

void obs_drop_add(uint32_t n)
{
    s_drop += n;
}
