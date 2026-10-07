/**
 * @file bsp_dwt.c
 * @date 2026-10-06
 * @brief 旧 DWT 计时实现。新代码优先 time_port。
 *
 * 节拍限制见 bsp_dwt.h 文件头。
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
 */

#include "bsp_dwt.h"

DWT_Time_t SysTime;
static uint32_t CPU_FREQ_Hz;       /* CPU 频率，单位 Hz */
static uint32_t CPU_FREQ_Hz_ms;    /* 每毫秒的 CPU 周期数 */
static uint32_t CPU_FREQ_Hz_us;    /* 每微秒的 CPU 周期数 */
static uint32_t CYCCNT_RountCount; /* CYCCNT 回绕次数 */
static uint32_t CYCCNT_LAST;       /* 上次采样的 CYCCNT */
uint64_t CYCCNT64;                 /* 64 位累计周期 */

static void DWT_CNT_Update(void);

/**
 * @brief 初始化 DWT 周期计数。
 * @param CPU_Freq_mHz CPU 主频，单位 MHz。
 */
void DWT_Init(uint32_t CPU_Freq_mHz)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    DWT->CYCCNT = 0u;

    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    CPU_FREQ_Hz = CPU_Freq_mHz * 1000000u;
    CPU_FREQ_Hz_ms = CPU_FREQ_Hz / 1000u;
    CPU_FREQ_Hz_us = CPU_FREQ_Hz / 1000000u;
    CYCCNT_RountCount = 0u;
}

/**
 * @brief 相对上次调用的时间差，单位秒。
 * @param cnt_last 调用方保存的上次 CYCCNT；本函数会写回当前值。
 * @return 时间差，单位 s。
 */
float DWT_GetDeltaT(uint32_t *cnt_last)
{
    uint32_t cnt_now = DWT->CYCCNT;
    float dt = (cnt_now - *cnt_last) / (float)CPU_FREQ_Hz;
    *cnt_last = cnt_now;

    DWT_CNT_Update();
    return dt;
}

/**
 * @brief 相对上次调用的时间差（双精度），单位秒。
 * @param cnt_last 调用方保存的上次 CYCCNT；本函数会写回当前值。
 * @return 时间差，单位 s。
 */
double DWT_GetDeltaT64(uint32_t *cnt_last)
{
    uint32_t cnt_now = DWT->CYCCNT;
    double dt = (cnt_now - *cnt_last) / (double)CPU_FREQ_Hz;
    *cnt_last = cnt_now;

    DWT_CNT_Update();
    return dt;
}

/**
 * @brief 刷新 SysTime（秒 / 毫秒 / 微秒分量）。
 */
void DWT_SysTimeUpdate(void)
{
    uint32_t cnt_now = DWT->CYCCNT;
    static uint64_t CNT_TEMP1, CNT_TEMP2, CNT_TEMP3;

    DWT_CNT_Update();

    CYCCNT64 = (uint64_t)CYCCNT_RountCount * UINT32_MAX + cnt_now;

    CNT_TEMP1 = CYCCNT64 / CPU_FREQ_Hz;
    CNT_TEMP2 = CYCCNT64 % CPU_FREQ_Hz;
    SysTime.s = (uint32_t)CNT_TEMP1;

    SysTime.ms = (uint16_t)(CNT_TEMP2 / CPU_FREQ_Hz_ms);
    CNT_TEMP3 = CNT_TEMP2 % CPU_FREQ_Hz_ms;

    SysTime.us = (uint16_t)(CNT_TEMP3 / CPU_FREQ_Hz_us);
}

/**
 * @brief 上电后的时间线，单位秒。
 */
float DWT_GetTimeline_s(void)
{
    DWT_SysTimeUpdate();
    return (float)SysTime.s + (float)SysTime.ms * 0.001f +
           (float)SysTime.us * 0.000001f;
}

/**
 * @brief 上电后的时间线，单位毫秒。
 */
float DWT_GetTimeline_ms(void)
{
    DWT_SysTimeUpdate();
    return (float)SysTime.s * 1000.0f + (float)SysTime.ms +
           (float)SysTime.us * 0.001f;
}

/**
 * @brief 上电后的时间线，单位微秒。
 */
uint64_t DWT_GetTimeline_us(void)
{
    DWT_SysTimeUpdate();
    return (uint64_t)SysTime.s * 1000000u +
           (uint64_t)SysTime.ms * 1000u +
           (uint64_t)SysTime.us;
}

/**
 * @brief 检测 CYCCNT 回绕并累加回合计数。
 */
static void DWT_CNT_Update(void)
{
    uint32_t cnt_now = DWT->CYCCNT;
    if (cnt_now < CYCCNT_LAST) {
        CYCCNT_RountCount++;
    }
    CYCCNT_LAST = cnt_now;
}

/**
 * @brief 忙等延时，单位秒。不要在电流环里用。
 * @param Delay 延时时间，单位 s。
 */
void DWT_Delay(float Delay)
{
    uint32_t tickstart = DWT->CYCCNT;
    float wait_cycles = Delay * (float)CPU_FREQ_Hz;

    while ((DWT->CYCCNT - tickstart) < wait_cycles) {
        __NOP();
    }
}

/**
 * @brief 忙等延时，单位微秒。不要在电流环里用。
 * @param us 延时时间，单位 µs。
 */
void DWT_Delay_us(uint32_t us)
{
    uint32_t tickstart = DWT->CYCCNT;
    uint32_t wait_cycles = us * CPU_FREQ_Hz_us;

    while ((DWT->CYCCNT - tickstart) < wait_cycles) {
        __NOP();
    }
}
