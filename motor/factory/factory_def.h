/**
 * @file factory_def.h
 * @date 2026-10-28
 * @brief 出厂自测序列：item 函数指针 + 执行器接口。
 *
 * 与 exp_runner 正交：factory 编排多项，item 内部可驱动实验或不做事。
 * 只允许在 2 kHz 外环节拍调用 factory_tick。
 */
#ifndef MOTOR_FACTORY_FACTORY_DEF_H
#define MOTOR_FACTORY_FACTORY_DEF_H

#include <stdint.h>

#include "motor_context.h"

#ifdef __cplusplus
extern "C" {
#endif

#define FACTORY_RUNNING  0u
#define FACTORY_DONE     1u
#define FACTORY_FAILED   2u

#define FACTORY_ST_IDLE   0u
#define FACTORY_ST_ENTER  1u
#define FACTORY_ST_RUN    2u
#define FACTORY_ST_EXIT   3u
#define FACTORY_ST_DONE   4u
#define FACTORY_ST_ABORT  5u

typedef struct factory_item_s factory_item_t;
typedef struct factory_seq_s factory_seq_t;

struct factory_item_s {
    const char *name;
    uint8_t (*enter)(motor_context_t *ctx);
    uint8_t (*tick)(motor_context_t *ctx, float dt_s);
    void (*exit)(motor_context_t *ctx);
    uint16_t timeout_ms;
};

struct factory_seq_s {
    const char *name;
    const factory_item_t *const *items;
    uint8_t n;
};

void factory_init(void);
uint8_t factory_start(const factory_seq_t *seq);
void factory_tick(motor_context_t *ctx, float dt_s);
void factory_abort(void);
uint8_t factory_state(void);
uint8_t factory_item_index(void);
uint32_t factory_elapsed_ms(void);

extern const factory_seq_t g_factory_stub_dryrun;

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_FACTORY_FACTORY_DEF_H */
