/**
 ******************************************************************************
 * @file    bsp_dwt.c
 * @author  Wang Hongxi
 * @version V1.1.0
 * @date    2022/3/8
 * @brief   基于DWT的高精度计时器驱动
 ******************************************************************************
 * @attention
 * 本模块使用DWT(Data Watchpoint and Trace)中的CYCCNT寄存器实现高精度计时，
 * 提供延时、时间差计算及系统时间线功能。
 * 注意：使用前需正确配置CPU主频，且仅适用于Cortex-M3/M4/M7等支持DWT的内核。
 ******************************************************************************
 */

#include "bsp_dwt.h"

DWT_Time_t SysTime;              // 系统时间结构体，存储秒、毫秒、微秒
static uint32_t CPU_FREQ_Hz;     // CPU频率，单位Hz
static uint32_t CPU_FREQ_Hz_ms;  // 每毫秒的CPU周期数
static uint32_t CPU_FREQ_Hz_us;  // 每微秒的CPU周期数
static uint32_t CYCCNT_RountCount; // CYCCNT溢出次数计数器
static uint32_t CYCCNT_LAST;     // 上一次CYCCNT值，用于溢出检测
uint64_t CYCCNT64;               // 64位组合的周期计数器

static void DWT_CNT_Update(void); // 更新周期计数器，处理溢出

/**
 * @brief 初始化DWT计时器
 * @param CPU_Freq_mHz CPU主频，单位MHz
 * @note 使能DWT外设，初始化周期计数器及频率参数
 */
void DWT_Init(uint32_t CPU_Freq_mHz)
{
    /* 使能DWT跟踪功能 */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    /* 清零周期计数器 */
    DWT->CYCCNT = 0u;

    /* 使能CYCCNT周期计数器 */
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    /* 计算时间转换因子 */
    CPU_FREQ_Hz = CPU_Freq_mHz * 1000000;        // 转换为Hz
    CPU_FREQ_Hz_ms = CPU_FREQ_Hz / 1000;         // 每毫秒周期数
    CPU_FREQ_Hz_us = CPU_FREQ_Hz / 1000000;      // 每微秒周期数
    CYCCNT_RountCount = 0;                       // 初始化溢出计数器
}

/**
 * @brief 计算与上次时间的时间差(单位：秒)
 * @param cnt_last 指向上次周期计数值的指针
 * @return float 时间差(s)
 */
float DWT_GetDeltaT(uint32_t *cnt_last)
{
    uint32_t cnt_now = DWT->CYCCNT;
    float dt = (cnt_now - *cnt_last) / (float)CPU_FREQ_Hz; // 计算时间差
    *cnt_last = cnt_now;                                   // 更新上次计数值

    DWT_CNT_Update(); // 处理计数器溢出
    return dt;
}

/**
 * @brief 计算与上次时间的时间差(双精度，单位：秒)
 * @param cnt_last 指向上次周期计数值的指针
 * @return double 时间差(s)
 */
double DWT_GetDeltaT64(uint32_t *cnt_last)
{
    uint32_t cnt_now = DWT->CYCCNT;
    double dt = (cnt_now - *cnt_last) / (double)CPU_FREQ_Hz; // 计算时间差
    *cnt_last = cnt_now;                                      // 更新上次计数值

    DWT_CNT_Update(); // 处理计数器溢出
    return dt;
}

/**
 * @brief 更新系统时间结构体
 * @note 将64位周期计数器转换为时分秒形式存储到SysTime
 */
void DWT_SysTimeUpdate(void)
{
    uint32_t cnt_now = DWT->CYCCNT;
    static uint64_t CNT_TEMP1, CNT_TEMP2, CNT_TEMP3;

    DWT_CNT_Update(); // 更新周期计数器状态

    // 组合64位周期计数值
    CYCCNT64 = (uint64_t)CYCCNT_RountCount * UINT32_MAX + cnt_now;
    
    // 计算秒、余数
    CNT_TEMP1 = CYCCNT64 / CPU_FREQ_Hz;
    CNT_TEMP2 = CYCCNT64 % CPU_FREQ_Hz;
    SysTime.s = CNT_TEMP1;
    
    // 计算毫秒、余数
    SysTime.ms = CNT_TEMP2 / CPU_FREQ_Hz_ms;
    CNT_TEMP3 = CNT_TEMP2 % CPU_FREQ_Hz_ms;
    
    // 计算微秒
    SysTime.us = CNT_TEMP3 / CPU_FREQ_Hz_us;
}

/**
 * @brief 获取当前系统时间线(单位：秒)
 * @return float 当前时间(s)
 */
float DWT_GetTimeline_s(void)
{
    DWT_SysTimeUpdate();
    return SysTime.s + SysTime.ms * 0.001f + SysTime.us * 0.000001f;
}

/**
 * @brief 获取当前系统时间线(单位：毫秒)
 * @return float 当前时间(ms)
 */
float DWT_GetTimeline_ms(void)
{
    DWT_SysTimeUpdate();
    return SysTime.s * 1000 + SysTime.ms + SysTime.us * 0.001f;
}

/**
 * @brief 获取当前系统时间线(单位：微秒)
 * @return uint64_t 当前时间(us)
 */
uint64_t DWT_GetTimeline_us(void)
{
    DWT_SysTimeUpdate();
    return SysTime.s * 1000000 + SysTime.ms * 1000 + SysTime.us;
}

/**
 * @brief 更新周期计数器状态，处理溢出
 * @note 当检测到CYCCNT回绕时，增加溢出计数器
 */
static void DWT_CNT_Update(void)
{
    uint32_t cnt_now = DWT->CYCCNT;
    if (cnt_now < CYCCNT_LAST)
        CYCCNT_RountCount++; // 检测到溢出，增加计数器
    CYCCNT_LAST = cnt_now;   // 更新最后计数值
}

/**
 * @brief 高精度延时函数
 * @param Delay 延时时间(秒)
 */
void DWT_Delay(float Delay)
{
    uint32_t tickstart = DWT->CYCCNT;
    float wait_cycles = Delay * CPU_FREQ_Hz; // 计算需等待的周期数

    // 等待直到经过足够的周期
    while ((DWT->CYCCNT - tickstart) < wait_cycles) 
    {
        __NOP(); // 避免编译器优化
    }
}
/**?
 * @brief 高精度微秒级延时
 * @param us 延时时间(微秒)
 */
void DWT_Delay_us(uint32_t us)
{
    uint32_t tickstart = DWT->CYCCNT;
    uint32_t wait_cycles = us * CPU_FREQ_Hz_us;  // 计算需等待的周期数
    
    // 等待直到经过足够的周期
    while ((DWT->CYCCNT - tickstart) < wait_cycles) 
    {
        __NOP(); // 避免编译器优化
    }
}