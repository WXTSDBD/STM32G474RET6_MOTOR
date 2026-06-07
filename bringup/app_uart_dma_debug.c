/**
 * @file app_uart_dma_debug.c
 * @brief UART_DMA_DEBUG RTOS 任务实现（强符号，覆盖 app_freertos.c 里的 __weak 空桩）
 *
 * 用途（Step 2 bringup）：
 *   - 在独立任务里周期组帧，经 LPUART1 以 JustFloat 协议发到 VOFA+
 *   - 当前用 HAL_UART_Transmit 阻塞发送；DMA / BSP 分层留待后续步骤
 *
 * VOFA+ 设置（TELEM_BRINGUP_INCLUDE_SEQ == 0，当前默认）：
 *   - 波特率 6000000，协议 JustFloat，CH_COUNT = 4
 *
 * VOFA+ 设置（TELEM_BRINGUP_INCLUDE_SEQ == 1）：
 *   - CH_COUNT = 5，通道 0 为序号 seq，建议在 VOFA+ 中隐藏
 */

#include "cmsis_os.h"
#include "usart.h"
#include <string.h>

/** 每帧数据通道数 k（不含 seq、不含帧尾），对应 VOFA+ 里「数据 float」的个数 */
#define TELEM_BRINGUP_K             4u

/**
 * 是否在帧首附带 32 位序号 seq（小端 uint32，VOFA+ 会当成第 0 路 float 显示）
 * 0：不发 seq，帧 = k 个 float + 帧尾（当前 bringup 默认，波形更干净）
 * 1：发 seq，用于后期丢包检测（相邻帧 seq 应差 1）
 */
#define TELEM_BRINGUP_INCLUDE_SEQ   0u

/** HAL_UART_Transmit 超时（ms）；24 字节 @ 6Mbps 实际约 0.03ms，10ms 仅作保险 */
#define TELEM_TX_TIMEOUT_MS         10u

#if TELEM_BRINGUP_INCLUDE_SEQ
/** 单帧总字节：seq(4) + k×float(4k) + 帧尾(4) */
#define TELEM_BRINGUP_FRAME_BYTES   (4u + TELEM_BRINGUP_K * 4u + 4u)
#else
/** 单帧总字节：k×float(4k) + 帧尾(4) */
#define TELEM_BRINGUP_FRAME_BYTES   (TELEM_BRINGUP_K * 4u + 4u)
#endif

/**
 * JustFloat 固定帧尾（4 字节），IEEE754 正无穷 0x7F800000 的小端序。
 * VOFA+ 靠扫描该序列判断一帧结束；缺了或错了会导致上位机缓冲胀死。
 */
static const uint8_t s_justfloat_tail[4] = {0x00u, 0x00u, 0x80u, 0x7fu};

/** 待发缓冲区；组帧完成后整段交给 HAL_UART_Transmit，发送期间内容须保持不变 */
static uint8_t s_frame[TELEM_BRINGUP_FRAME_BYTES];

#if TELEM_BRINGUP_INCLUDE_SEQ
/** 小帧序号，每成功组一帧自增 1；仅在 TELEM_BRINGUP_INCLUDE_SEQ==1 时写入帧首 */
static uint32_t s_seq;
#endif

/**
 * @brief 组一帧 JustFloat 测试数据并通过 LPUART1 阻塞发出
 *
 * 帧布局（INCLUDE_SEQ==0）：
 *   [ch0 float][ch1 float][ch2 float][ch3 float][tail 4B]
 * 当前测试值固定为 1.0、2.0、3.0、4.0，仅用于验证链路与协议。
 */
static void telem_bringup_send_test_frame(void)
{
    /** 本帧各通道 IEEE754 浮点值（小端写入 s_frame） */
    float vals[4];
    /** 指向 s_frame 内当前写入位置，组帧时顺序后移 */
    uint8_t *p;

    vals[0] = 1.0f;
    vals[1] = 2.0f;
    vals[2] = 3.0f;
    vals[3] = 4.0f;

    p = s_frame;

#if TELEM_BRINGUP_INCLUDE_SEQ
    {
        /** 本帧序号快照；写入后 s_seq 加 1 供下一帧使用 */
        uint32_t seq = s_seq++;
        memcpy(p, &seq, sizeof(seq));
        p += sizeof(seq);
    }
#endif

    memcpy(p, vals, sizeof(vals));
    p += sizeof(vals);
    memcpy(p, s_justfloat_tail, sizeof(s_justfloat_tail));

    (void)HAL_UART_Transmit(&hlpuart1, s_frame, TELEM_BRINGUP_FRAME_BYTES, TELEM_TX_TIMEOUT_MS);
}

/**
 * @brief CubeMX 创建的 UART_DMA_DEBUG 任务入口（线程名 UART_DMA_DEBUG，低优先级）
 * @param argument RTOS 传入，未使用
 *
 * 每 100ms 发一帧（约 10Hz），便于 VOFA+ 观察；后期可改为更短周期或改 DMA。
 */
void UART_DMA_DEBUG_TASK(void *argument)
{
    (void)argument;

    for (;;) {
        telem_bringup_send_test_frame();
        osDelay(100);
    }
}
