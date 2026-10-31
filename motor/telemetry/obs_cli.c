/**
 * @file obs_cli.c
 * @date 2026-10-31
 * @brief LPUART1 RX DMA 收字节；RTOS 听窗拼命令并换通道。
 *
 * 半双工听窗停发；换表只在本任务。成功把通道数写入 g_obs_cli_marker。
 */
#include "telemetry/obs_reg.h"
#include "telemetry/obs_preset.h"
#include "telemetry/obs_cli.h"

#include "usart.h"
#include "cmsis_os.h"
#include "motor_params_m1.h"

#include <stdlib.h>
#include <string.h>

extern UART_HandleTypeDef hlpuart1;
extern float g_obs_cli_marker;
extern float g_obs_rx_bytes;

#define OBS_CLI_LINE_MAX 80u
#define OBS_RX_DMA_SZ    128u
#define OBS_RX_Q_SZ      256u

static char s_line[OBS_CLI_LINE_MAX];
static uint16_t s_line_len;

static uint8_t s_rx_dma[OBS_RX_DMA_SZ];
static volatile uint16_t s_rx_dma_pos;

static uint8_t s_rx_q[OBS_RX_Q_SZ];
static volatile uint16_t s_rx_q_w;
static volatile uint16_t s_rx_q_r;

static volatile uint8_t s_telem_gate = 1u;
static uint8_t s_preset_id;

uint8_t obs_cli_telem_enabled(void)
{
    return s_telem_gate;
}

/**
 * @brief 启动/重启 Idle+Circular 接收。
 */
static void obs_cli_rx_start(void)
{
    s_rx_dma_pos = 0u;
    if (HAL_UARTEx_ReceiveToIdle_DMA(&hlpuart1, s_rx_dma, OBS_RX_DMA_SZ) != HAL_OK) {
        return;
    }
    if (hlpuart1.hdmarx != 0) {
        __HAL_DMA_DISABLE_IT(hlpuart1.hdmarx, DMA_IT_HT);
    }
}

/**
 * @brief marker：成功写通道数，失败写负数。
 */
static void obs_cli_mark_rc(uint8_t rc)
{
    g_obs_cli_marker = (rc == 0u) ? (float)obs_session_n() : -1.0f;
}

/**
 * @brief OBS SET a,b,c
 */
static void obs_cli_do_set(char *arg)
{
    const char *names[OBS_CH_MAX];
    uint8_t n = 0u;
    char *tok;
    uint8_t decim = obs_session_decim();

    if (decim == 0u) {
        decim = 2u;
    }
    for (tok = strtok(arg, ","); (tok != 0) && (n < OBS_CH_MAX); tok = strtok(0, ",")) {
        while (*tok == ' ') {
            tok++;
        }
        names[n++] = tok;
    }
    obs_cli_mark_rc(obs_session_set(names, n, decim));
}

/**
 * @brief OBS RATE 1|2|4（按当前预设重挂）。
 */
static void obs_cli_do_rate(char *arg)
{
    long v = strtol(arg, 0, 10);

    if ((v != 1) && (v != 2) && (v != 4)) {
        g_obs_cli_marker = -2.0f;
        return;
    }
    obs_cli_mark_rc(obs_preset_apply(s_preset_id, (uint8_t)v));
}

/**
 * @brief OBS PRESET <id>
 */
static void obs_cli_do_preset(char *arg)
{
    uint8_t id = (uint8_t)strtol(arg, 0, 10);
    uint8_t rc = obs_preset_apply(id, obs_session_decim());

    if (rc == 0u) {
        s_preset_id = id;
    }
    obs_cli_mark_rc(rc);
}

/**
 * @brief 一行命令分发。
 */
static void obs_cli_on_line(char *line)
{
    while ((*line == ' ') || (*line == '\t')) {
        line++;
    }
    if (strncmp(line, "OBS SET ", 8) == 0) {
        obs_cli_do_set(line + 8);
        return;
    }
    if (strncmp(line, "OBS RATE ", 9) == 0) {
        obs_cli_do_rate(line + 9);
        return;
    }
    if (strncmp(line, "OBS PRESET ", 11) == 0) {
        obs_cli_do_preset(line + 11);
        return;
    }
    if (strcmp(line, "OBS LIST") == 0) {
        g_obs_cli_marker = (float)obs_count();
    }
}

/**
 * @brief DMA 字节入队（ISR）。
 */
static void obs_cli_q_push(uint8_t b)
{
    uint16_t n = (uint16_t)((s_rx_q_w + 1u) % OBS_RX_Q_SZ);

    if (n == s_rx_q_r) {
        return;
    }
    s_rx_q[s_rx_q_w] = b;
    s_rx_q_w = n;
    g_obs_rx_bytes += 1.0f;
}

/**
 * @brief DMA 写出下标 Size；拷走新增字节。
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    uint16_t end = Size;

    if ((huart == 0) || (huart->Instance != LPUART1)) {
        return;
    }
    if (end >= OBS_RX_DMA_SZ) {
        end = 0u;
    }
    while (s_rx_dma_pos != end) {
        obs_cli_q_push(s_rx_dma[s_rx_dma_pos]);
        s_rx_dma_pos++;
        if (s_rx_dma_pos >= OBS_RX_DMA_SZ) {
            s_rx_dma_pos = 0u;
        }
    }
}

/**
 * @brief 接收出错后重新挂上 DMA。
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if ((huart != 0) && (huart->Instance == LPUART1)) {
        obs_cli_rx_start();
    }
}

/**
 * @brief 丢掉队列里未读字节。
 */
static void obs_cli_q_flush(void)
{
    s_rx_q_r = s_rx_q_w;
    s_line_len = 0u;
}

/**
 * @brief 拼一行：换行提交，可打印字符入缓冲，其余丢弃。
 */
static void obs_cli_feed_byte(uint8_t b)
{
    if ((b == (uint8_t)'\n') || (b == (uint8_t)'\r')) {
        if (s_line_len == 0u) {
            return;
        }
        s_line[s_line_len] = '\0';
        obs_cli_on_line(s_line);
        s_line_len = 0u;
        return;
    }
    if ((b < 0x20u) || (b > 0x7Eu)) {
        s_line_len = 0u;
        return;
    }
    if (s_line_len + 1u >= OBS_CLI_LINE_MAX) {
        s_line_len = 0u;
        return;
    }
    s_line[s_line_len++] = (char)b;
}

/**
 * @brief 排空软件队列。
 */
static void obs_cli_drain_q(void)
{
    while (s_rx_q_r != s_rx_q_w) {
        uint8_t b = s_rx_q[s_rx_q_r];
        s_rx_q_r = (uint16_t)((s_rx_q_r + 1u) % OBS_RX_Q_SZ);
        obs_cli_feed_byte(b);
    }
}

/**
 * @brief 开机挂配方默认预设；周期听窗收 OBS 命令。
 */
void CLI_Shell_Task(void *argument)
{
    uint32_t i;
    uint8_t decim = (uint8_t)M1_TELEM_BRINGUP_DECIMATION;

    (void)argument;
    if (decim == 0u) {
        decim = 2u;
    }

    obs_reg_init();
    s_preset_id = obs_preset_default_id();
    obs_cli_mark_rc(obs_preset_apply(s_preset_id, decim));
    s_line_len = 0u;
    s_rx_q_w = 0u;
    s_rx_q_r = 0u;
    s_telem_gate = 1u;
    obs_cli_rx_start();

    for (;;) {
        s_telem_gate = 0u;
        osDelay(5);
        obs_cli_q_flush();
        for (i = 0u; i < 50u; i++) {
            obs_cli_drain_q();
            osDelay(2);
        }
        s_telem_gate = 1u;
        osDelay(200);
    }
}
