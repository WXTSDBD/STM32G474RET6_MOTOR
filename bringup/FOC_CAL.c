#include "FOC_CAL.h"
#include <math.h> 
#include "gpio.h"       // 包含GPIO（通用输入输出）功能的头文件
#include "arm_math.h"   // 包含ARM数学库的头文件，用于优化数学运算
#include "tim.h"
//#include "as5047.h"
#include "bsp_dwt.h"
//#include "ADC_TASK.H"
#include "trans.h"
typedef struct
{
    float speed_target;
    float I_target;
    float pos_target;
    float weak_big;
    float weak_small;
    float weak;
    float angle_raw;
    float angle_add;
    int state;
    float padel_now;
    float Uq, Uq_ramp;
    
    
} foc_control;
int elect_pair_num = 14;//电极对数，2312s电极对数为12，tmotor_u3,7
foc_control motor_now_TIM1;
foc_control motor_now_TIM8;
void angle_init()
{
    as5047_spi1.add=2.1;
    as5047_spi3.add=1.2;
}
#define Iu ADC_DATA.IA
#define Iv ADC_DATA.IB
#define Iw ADC_DATA.IC
#define PWM_Period 5300
#define _PI 3.14159265359
#define _PI_2 1.57079632679
#define _PI_3 1.0471975512
#define _2PI 6.28318530718
#define _3PI_2 4.71238898038
#define _PI_6 0.52359877559
#define _SQRT3 1.73205080757

// 电机参数
#define MOTOR_KV  220.0f  // 电机KV值（RPM/V）
#define MOTOR_KE  (60.0f / (2.0f * _PI * MOTOR_KV)) // 反电动势常数（V·s/rad）
#define MOTOR_KT  MOTOR_KE // 转矩常数（Nm/A）

angle as5047_spi1;
angle as5047_spi3;
float angle_el=0;
int delay_time=1;
float uq_intput=2;
int limit=30000;
float speed_now;
int count;
//int i=0;
/***************************************************************************
* 功能：无感开环控制任务
* 实现逻辑：
* 1. 读取编码器原始角度数据
* 2. 获取处理后的角度值
* 3. 计算并更新电角度
* 4. 设置相电压
* 5. 计算当前速度
* 6. 更新上次角度值
*****************************************************************************/
void VF_OPENLOOP_TASK(TIM_HandleTypeDef *htim,angle *angle_now)
{
    // 设置相电压
    setPhaseVoltage(htim, uq_intput,0, angle_el); // 设置相电压，ud_input为输入电压，angle_el为电角度

    // 更新电角度估计值
    angle_el = angle_el + angle_now->add; // 对电角度进行补偿或校准
}
