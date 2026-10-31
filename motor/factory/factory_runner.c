/**
 * @file factory_runner.c
 * @date 2026-10-28
 * @brief 出厂自测执行器空壳（BUILD_FACTORY_TEST 已下线，恒 IDLE）。
 *
 * 节拍：只从 2 kHz 外环调用 factory_tick；未接真序列。
 */
#include "factory_def.h"

#include <stddef.h>

void factory_init(void) {}

uint8_t factory_start(const factory_seq_t *seq)
{
    (void)seq;
    return 1u;
}

void factory_tick(motor_context_t *ctx, float dt_s)
{
    (void)ctx;
    (void)dt_s;
}

void factory_abort(void) {}

uint8_t factory_state(void)
{
    return FACTORY_ST_IDLE;
}

uint8_t factory_item_index(void)
{
    return 0u;
}

uint32_t factory_elapsed_ms(void)
{
    return 0u;
}

const factory_seq_t g_factory_stub_dryrun = { "stub_off", NULL, 0u };
