/**
  * @file   Sliding_Window_Filter.h
  * @brief  滑窗滤波器头文件（包含按钮防抖动滤波）
  * @note   1. 为每个滤波器提供专属初始化函数
  *         2. 窗口大小在初始化时固定
  *         3. 避免运行时动态调整，减少函数调用
  *         4. 新增按钮防抖动滤波器
  * @author Your Name
  * @date   2023-10-15
  */
#ifndef SLIDING_WINDOW_FILTER_H
#define SLIDING_WINDOW_FILTER_H

#define WINDOW_SIZE 20
#define BUTTON_FILTER_SIZE 20

#include <stdint.h>

/**
  * @brief  滑窗滤波器结构体
  * @note   1. 窗口大小在初始化时固定
  *         2. 无需运行时动态调整
  */
typedef struct {
    float buffer[WINDOW_SIZE];  // 数据缓冲区
    float sum;                  // 当前窗口总和
    int index;                  // 当前写入位置
    int max_indexes[2];         // 最大两个值的索引
    int min_indexes[2];         // 最小两个值的索引
} MovingAverageFilter;

/**
  * @brief  按钮防抖动滤波器结构体
  * @note   1. 连续多次相同信号才确认状态变化
  *         2. 防止机械按钮抖动造成的误触发
  */
typedef struct {
    uint8_t current_state;       // 当前稳定状态
    uint8_t temp_state;          // 临时状态
    uint8_t confirm_count;       // 确认计数
    uint8_t threshold;           // 置信阈值(默认10)
    uint8_t max_count;           // 最大计数值
} ButtonFilter;

/**
  * @brief  统一的滑窗滤波器初始化函数
  * @note   1. 通用初始化函数，设置窗口大小
  *         2. 窗口大小在初始化时固定
  * @param  filter: 滤波器结构体指针
  * @param  window_size: 窗口大小(1-20)
  * @retval 无
  */
void Filter_Init(MovingAverageFilter *filter, uint8_t window_size);

/**
  * @brief  按钮滤波器初始化函数
  * @param  filter: 按钮滤波器结构体指针
  * @param  init_state: 初始状态
  * @param  threshold: 确认阈值
  * @retval 无
  */
void Button_Filter_Init(ButtonFilter *filter, uint8_t init_state, uint8_t threshold);

/**
  * @brief  处理新数据并返回滑窗滤波结果
  * @param  filter: 滤波器结构体指针
  * @param  new_value: 新的输入值
  * @retval 滤波后的平均值
  */
float Filter_Process(MovingAverageFilter *filter, float new_value);

/**
  * @brief  按钮滤波处理
  * @note   连续threshold次相同信号才更新状态
  * @param  filter: 按钮滤波器结构体指针
  * @param  new_state: 新的按钮状态
  * @retval 滤波后的稳定状态
  */
uint8_t Button_Filter_Process(ButtonFilter *filter, uint8_t new_state);

/**
  * @brief  初始化所有滤波器（总初始化函数）
  * @note   1. 速度滤波器窗口大小为10
  *         2. 一次性初始化所有滤波器
  * @retval 无
  */
void Filter_Init_All(void);

/**
  * @brief  遥控器和按钮处理任务
  * @note   放在低速时间片执行(如case 3)
  * @retval 无
  */
void Remote_Button_Process_Task(void);
float ADC_Filter_Process(float new_adc_value,MovingAverageFilter *filter) ;
// 全局滤波器实例
extern MovingAverageFilter speed_filter;
extern MovingAverageFilter x_filter_good;
extern MovingAverageFilter y_filter_good;
extern MovingAverageFilter distance_filter;
extern MovingAverageFilter speed_filter_tim8;
extern MovingAverageFilter speed_filter_tim1;

// 新增ADC滤波器实例声明
extern MovingAverageFilter adc_button_filter;
extern MovingAverageFilter adc_padel;
// 按钮和遥控器滤波器实例
extern ButtonFilter button_mode_filter;

#endif /* SLIDING_WINDOW_FILTER_H */


