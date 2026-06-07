/**
  * @file   Sliding_Window_Filter.c
  * @brief  滑窗滤波器实现（包含按钮防抖动滤波）
  * @note   1. 为每个滤波器提供专属初始化函数
  *         2. 窗口大小在初始化时固定
  *         3. 避免运行时动态调整，减少函数调用
  *         4. 新增按钮防抖动滤波器
  * @author Your Name
  * @date   2023-10-15
  */
#include "Sliding_Window_Filter.h"
#include <string.h>
#include "arm_math.h"
#include "main.h"  // 需要包含main.h以使用HAL_GPIO_ReadPin等函数

// 外部变量声明（根据你的实际文件调整）
extern int buttun_mode;
// 假设你有这些外部变量，如果没有请删除或调整
// extern float remote_ble_data_left_speed, remote_ble_data_right_speed;

/**
  * @brief  统一的滑窗滤波器初始化函数
  * @note   窗口大小参数保留用于兼容性，实际使用WINDOW_SIZE
  */
void Filter_Init(MovingAverageFilter *filter, uint8_t window_size) {
    memset(filter->buffer, 0, sizeof(filter->buffer));
    filter->sum = 0.0f;
    filter->index = 0;
    filter->max_indexes[0] = filter->max_indexes[1] = 0;
    filter->min_indexes[0] = filter->min_indexes[1] = 0;
}

/**
  * @brief  按钮滤波器初始化函数
  */
void Button_Filter_Init(ButtonFilter *filter, uint8_t init_state, uint8_t threshold) {
    filter->current_state = init_state;
    filter->temp_state = init_state;
    filter->confirm_count = threshold;  // 初始直接确认为稳定状态
    filter->threshold = threshold;
    filter->max_count = threshold + 5;  // 最大计数为阈值+5
}

/**
  * @brief  处理新数据并返回滑窗滤波结果（增加异常值过滤）
  * @note   1. 窗口大小已固定，无需运行时检查
  *         2. 直接使用固定窗口大小进行计算
  *         3. 减少函数调用，提升性能
  *         4. 新增：过滤绝对值大于800的数据（异常值）
  */
float Filter_Process(MovingAverageFilter *filter, float new_value) {
    // 异常值过滤
    if (new_value > 800.0f || new_value < -800.0f) {
        return filter->sum / WINDOW_SIZE;
    }
    
    // 轻量级智能异常值处理
    float current_avg = filter->sum / WINDOW_SIZE;
    float dynamic_threshold = fabsf(current_avg) * 0.3f + 30.0f;
    
    if (fabsf(new_value - current_avg) > dynamic_threshold) {
        new_value = current_avg + (new_value - current_avg) * 0.15f;
    }
    
    // 减去即将被替换的旧值
    filter->sum -= filter->buffer[filter->index];
    // 添加新值并更新总和
    filter->sum += new_value;
    filter->buffer[filter->index] = new_value;
    // 更新索引（环形缓冲）
    filter->index = (filter->index + 1) % WINDOW_SIZE;
	// 在Filter_Process函数最后，返回前加上：
static float last_filtered = 0.0f;
float final_speed = 0.98f * last_filtered + 0.02f * (filter->sum / WINDOW_SIZE);
last_filtered = final_speed;
return final_speed;
    // 返回平均值
    return filter->sum / WINDOW_SIZE;
}

/**
  * @brief  按钮滤波处理
  * @note   连续threshold次相同信号才更新状态
  */
uint8_t Button_Filter_Process(ButtonFilter *filter, uint8_t new_state) {
    if (new_state == filter->temp_state) {
        // 相同信号，计数增加
        if (filter->confirm_count < filter->max_count) {
            filter->confirm_count++;
        }
    } else {
        // 不同信号，重置计数
        filter->temp_state = new_state;
        filter->confirm_count = 1;
    }
    
    // 达到阈值，更新稳定状态
    if (filter->confirm_count >= filter->threshold) {
        filter->current_state = filter->temp_state;
    }
    
    return filter->current_state;
}
float ADC_Filter_Process(float new_adc_value,MovingAverageFilter *filter) {
    
    // 减去即将被替换的旧值
    filter->sum -= filter->buffer[filter->index];
    // 添加新值并更新总和
    filter->sum += new_adc_value;
    filter->buffer[filter->index] = new_adc_value;
    // 更新索引（环形缓冲）
    filter->index = (filter->index + 1) % WINDOW_SIZE;
    // 返回平均值
    return filter->sum / WINDOW_SIZE;
}
/**
  * @brief  遥控器和按钮处理任务
  * @note   放在低速时间片执行(如case 3)
  */
//void Remote_Button_Process_Task(void) {
//    // 1. 读取按钮原始状态
////    uint8_t raw_button_state = HAL_GPIO_ReadPin(BUTTUN_MODE_GPIO_Port, BUTTUN_MODE_Pin);
//    
//    // 2. 按钮滤波处理
//    Button_Filter_Process(&button_mode_filter, raw_button_state);
//    
//    // 3. 使用滤波后的状态
//    buttun_mode = button_mode_filter.current_state;
//    
//    // 4. 这里可以添加遥控器数据的滑窗滤波处理
//    // 例如：remote_ble_data.left_speed = Filter_Process(&remote_left_filter, raw_left_speed);
//}

// ========== 全局滤波器实例定义 ==========

// 原有滤波器实例
MovingAverageFilter speed_filter;
MovingAverageFilter x_filter_good, y_filter_good, distance_filter;
MovingAverageFilter speed_filter_tim8, speed_filter_tim1;
MovingAverageFilter adc_button_filter;
MovingAverageFilter adc_padel;
// 新增按钮滤波器实例
ButtonFilter button_mode_filter;

/**
  * @brief  初始化所有滤波器（总初始化函数）
  * @note   使用固定窗口大小WINDOW_SIZE
  */
void Filter_Init_All(void) {
    // 初始化原有滑窗滤波器
    Filter_Init(&speed_filter, WINDOW_SIZE);
    Filter_Init(&x_filter_good, WINDOW_SIZE);
    Filter_Init(&y_filter_good, WINDOW_SIZE);
    Filter_Init(&distance_filter, WINDOW_SIZE);
    Filter_Init(&speed_filter_tim8, WINDOW_SIZE);
    Filter_Init(&speed_filter_tim1, WINDOW_SIZE);
    
    // 初始化按钮滤波器（默认模式1，阈值10次）
    Button_Filter_Init(&button_mode_filter, 1, 10);
}