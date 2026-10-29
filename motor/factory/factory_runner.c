/**
 * @file factory_runner.c
 * @date 2026-10-28
 * @brief 出厂自测执行器：按序 enter → tick → exit，支持超时与中止。
 *
 * 节拍：只从 2 kHz 外环调用 factory_tick。未 start 时为 IDLE 空转。
 */
#include "factory_def.h"

#include <stddef.h>

#include "build_config.h"

#if BUILD_FACTORY_TEST

static const factory_seq_t *s_seq;
static uint8_t s_state = FACTORY_ST_IDLE;
static uint8_t s_item_index;
static uint32_t s_elapsed_ms;
static float s_elapsed_acc_s;

/**
 * @brief 清内部状态到 IDLE（不碰电机指令）。
 */
static void factory_reset_idle(void)
{
    s_seq = NULL;
    s_state = FACTORY_ST_IDLE;
    s_item_index = 0u;
    s_elapsed_ms = 0u;
    s_elapsed_acc_s = 0.0f;
}

/**
 * @brief 进入当前 item 的 ENTER；失败则 ABORT。
 */
static void factory_do_enter(motor_context_t *ctx)
{
    const factory_item_t *item;

    if ((s_seq == NULL) || (s_item_index >= s_seq->n)) {
        s_state = FACTORY_ST_DONE;
        return;
    }
    item = s_seq->items[s_item_index];
    s_elapsed_ms = 0u;
    s_elapsed_acc_s = 0.0f;
    s_state = FACTORY_ST_ENTER;
    if ((item != NULL) && (item->enter != NULL)) {
        if (item->enter(ctx) != 0u) {
            s_state = FACTORY_ST_ABORT;
            return;
        }
    }
    s_state = FACTORY_ST_RUN;
}

void factory_init(void)
{
    factory_reset_idle();
}

uint8_t factory_start(const factory_seq_t *seq)
{
    if ((seq == NULL) || (seq->items == NULL) || (seq->n == 0u)) {
        return 1u;
    }
    s_seq = seq;
    s_item_index = 0u;
    s_elapsed_ms = 0u;
    s_elapsed_acc_s = 0.0f;
    factory_do_enter(NULL);
    if (s_state == FACTORY_ST_ABORT) {
        return 1u;
    }
    return 0u;
}

void factory_abort(void)
{
    if (s_state == FACTORY_ST_IDLE || s_state == FACTORY_ST_DONE) {
        return;
    }
    if ((s_seq != NULL) && (s_item_index < s_seq->n)) {
        const factory_item_t *item = s_seq->items[s_item_index];
        if ((item != NULL) && (item->exit != NULL)) {
            item->exit(NULL);
        }
    }
    s_state = FACTORY_ST_ABORT;
}

void factory_tick(motor_context_t *ctx, float dt_s)
{
    const factory_item_t *item;
    uint8_t tr;

    if (s_state != FACTORY_ST_RUN) {
        return;
    }
    if ((s_seq == NULL) || (s_item_index >= s_seq->n)) {
        s_state = FACTORY_ST_DONE;
        return;
    }
    item = s_seq->items[s_item_index];
    if (item == NULL) {
        s_state = FACTORY_ST_ABORT;
        return;
    }

    s_elapsed_acc_s += dt_s;
    while (s_elapsed_acc_s >= 0.001f) {
        s_elapsed_acc_s -= 0.001f;
        s_elapsed_ms++;
    }
    if ((item->timeout_ms != 0u) && (s_elapsed_ms >= (uint32_t)item->timeout_ms)) {
        if (item->exit != NULL) {
            item->exit(ctx);
        }
        s_state = FACTORY_ST_ABORT;
        return;
    }

    tr = FACTORY_DONE;
    if (item->tick != NULL) {
        tr = item->tick(ctx, dt_s);
    }
    if (tr == FACTORY_RUNNING) {
        return;
    }
    if (tr == FACTORY_FAILED) {
        if (item->exit != NULL) {
            item->exit(ctx);
        }
        s_state = FACTORY_ST_ABORT;
        return;
    }

    s_state = FACTORY_ST_EXIT;
    if (item->exit != NULL) {
        item->exit(ctx);
    }
    s_item_index++;
    if (s_item_index >= s_seq->n) {
        s_state = FACTORY_ST_DONE;
        return;
    }
    factory_do_enter(ctx);
}

uint8_t factory_state(void)
{
    return s_state;
}

uint8_t factory_item_index(void)
{
    return s_item_index;
}

uint32_t factory_elapsed_ms(void)
{
    return s_elapsed_ms;
}

/* ---- stub dry-run：三项只计 tick，验证状态机 ---- */

static uint16_t s_stub_ticks[3];

static uint8_t stub_enter(motor_context_t *ctx)
{
    (void)ctx;
    if (s_item_index < 3u) {
        s_stub_ticks[s_item_index] = 0u;
    }
    return 0u;
}

static uint8_t stub_tick(motor_context_t *ctx, float dt_s)
{
    (void)ctx;
    (void)dt_s;
    if (s_item_index >= 3u) {
        return FACTORY_DONE;
    }
    s_stub_ticks[s_item_index]++;
    /* 2 kHz × 4 tick ≈ 2 ms，远小于 timeout */
    if (s_stub_ticks[s_item_index] >= 4u) {
        return FACTORY_DONE;
    }
    return FACTORY_RUNNING;
}

static const factory_item_t s_stub0 = { "stub0", stub_enter, stub_tick, NULL, 1000u };
static const factory_item_t s_stub1 = { "stub1", stub_enter, stub_tick, NULL, 1000u };
static const factory_item_t s_stub2 = { "stub2", stub_enter, stub_tick, NULL, 1000u };

static const factory_item_t *const s_stub_items[] = {
    &s_stub0, &s_stub1, &s_stub2,
};

const factory_seq_t g_factory_stub_dryrun = {
    "stub_dryrun",
    s_stub_items,
    3u,
};

#else /* !BUILD_FACTORY_TEST */

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
uint8_t factory_state(void) { return FACTORY_ST_IDLE; }
uint8_t factory_item_index(void) { return 0u; }
uint32_t factory_elapsed_ms(void) { return 0u; }

const factory_seq_t g_factory_stub_dryrun = { "stub_off", NULL, 0u };

#endif /* BUILD_FACTORY_TEST */
