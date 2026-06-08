#include "trans.h"
#include "FOC_CAL.h"
//#include <math.h> 
#include "gpio.h"       // 包含GPIO（通用输入输出）功能的头文件
//#include "arm_math.h"   // 包含ARM数学库的头文件，用于优化数学运算
#include "tim.h"
#include "bsp_dwt.h"
#include "arm_math.h"
//#include "ADC_TASK.H"
#include "main.h"
//#include "usart.h"
#define voltage_power_supply 24 // 后续改为adc采集的母线电压
#define PWM_Period 4000
#define _PI 3.14159265359f
#define _PI_2 1.57079632679f
#define _PI_3 1.0471975512f
#define _2PI 6.28318530718f
#define _3PI_2 4.71238898038f
#define _PI_6 0.52359877559f
#define _SQRT3 1.73205080757f
float t_s,t_last;

void Park_Transform(float Ialpha, float Ibeta, float theta, float *Id, float *Iq)
{
    float sin_val = arm_sin_f32(theta); // 计算theta的正弦值
    float cos_val = arm_cos_f32(theta); // 计算theta的余弦值
    *Id =  Ialpha * cos_val + Ibeta * sin_val; // 计算d轴电流分量
    *Iq = -Ialpha * sin_val + Ibeta * cos_val; // 计算q轴电流分量
}

void Anti_Park_Transform(float mod_d, float mod_q, float theta, float *mod_alpha, float *mod_beta)
{
    float sin_val = arm_sin_f32(theta); // 计算theta的正弦值
    float cos_val = arm_cos_f32(theta); // 计算theta的余弦值
    *mod_alpha = mod_d * cos_val - mod_q * sin_val; // 计算alpha轴调制分量
    *mod_beta  = mod_d * sin_val + mod_q * cos_val; // 计算beta轴调制分量
}

void Clarke_Transform(float Ia, float Ib, float Ic, float *Ialpha, float *Ibeta)
{
    *Ialpha = Ia; // alpha轴分量等于a相电流
    *Ibeta  = (Ib - Ic) * sqrt(3); // beta轴分量由b相和c相电流计算得到
}

// 角度归一化函数
float _normalizeAngle(float angle) {
    float a = fmod(angle, _2PI);
    if (a < 0.0f) {
        a += _2PI;
    }
    return a;
}


int cct=0;
//// FOC核心函数：输入Ud、Uq和电角度，输出PWM
void setPhaseVoltage(TIM_HandleTypeDef *htim, float Uq, float Ud, float angle_el) {
    /* 电压前处理 */
    float Uref;
    float U_alpha, U_beta;
    float T1, T2, T0;
    float Ta, Tb, Tc;
    int sector;

    // 1. 反Park变换
    U_alpha = Ud * arm_cos_f32(angle_el) - Uq * arm_sin_f32(angle_el);
    U_beta = Ud * arm_sin_f32(angle_el) + Uq * arm_cos_f32(angle_el);

//    // 2. 电压矢量幅值计算与限幅
//    Uref = sqrt(U_alpha*U_alpha + U_beta*U_beta) / voltage_power_supply;
//    if(Uref> 0.577)Uref= 0.577;                     			//六边形的内切圆(SVPWM最大不失真旋转电压矢量赋值)根号3/3
//	if(Uref<-0.577)Uref=-0.577; 
	
	
	//2.1过调剂算法拓展
	if (Uref > 1.0f) {
    // 过调制区域II（六步方波逼近）
    Uref = 1.0f; 
    // 设置过调制标志，后续切换至六步方波模式
} else if (Uref > 0.577f) { 
    // 过调制区域I（调整矢量作用时间）
    Uref = 0.577f + (Uref - 0.577f) * 0.8f; // 示例：非线性扩展
}
    // 3. 扇区判断（带角度修正）
	
	
  // 正确方法：通过atan2计算电压矢量的实际角度
float angle_ref =0; 
	//atan2f(U_beta, U_alpha);
angle_ref = _normalizeAngle(angle_ref); // 归一化到0-2π
sector = (int)(angle_ref / _PI_3) % 6 + 1;
    // 4. 矢量作用时间计算
    float theta = _normalizeAngle(angle_ref - (sector-1)*_PI_3);
    T1 = _SQRT3 * arm_sin_f32(sector*_PI_3 - angle_ref) * Uref;
    T2 = _SQRT3 * arm_sin_f32(theta) * Uref;
    // 原T0计算改为：
T0 = 1.0f - T1 - T2;
if (T0 < 0) { // 过调制区域处理
    T0 = 0;
    T1 = T1 / (T1 + T2); // 归一化时间分配
    T2 = T2 / (T1 + T2);
}

    /* 七段式时间分配（每个扇区插入T0/2零矢量）*/
    switch(sector) {
        // 扇区1：U0→U1→U2→U7→U2→U1→U0
        case 1:
            Ta = T1 + T2 + T0/2; // U相：前T0/2 + T1 + T2 + 后T0/2
            Tb = T2 + T0/2;      // V相：前T0/2 + T2 + 后T0/2
            Tc = T0/2;           // W相：始终为T0/2
            break;
        
        // 扇区2：U0→U3→U2→U7→U2→U3→U0
        case 2:
            Ta = T1 + T0/2;
            Tb = T1 + T2 + T0/2;
            Tc = T0/2;
            break;

        // 扇区3：U0→U3→U4→U7→U4→U3→U0
        case 3:
            Ta = T0/2;
            Tb = T1 + T2 + T0/2;
            Tc = T2 + T0/2;
            break;

        // 扇区4：U0→U5→U4→U7→U4→U5→U0
        case 4:
            Ta = T0/2;
            Tb = T1 + T0/2;
            Tc = T1 + T2 + T0/2;
            break;

        // 扇区5：U0→U5→U6→U7→U6→U5→U0
        case 5:
            Ta = T2 + T0/2;
            Tb = T0/2;
            Tc = T1 + T2 + T0/2;
            break;

        // 扇区6：U0→U1→U6→U7→U6→U1→U0
        case 6:
            Ta = T1 + T2 + T0/2;
            Tb = T0/2;
            Tc = T1 + T0/2;
            break;

        default: // 错误处理
            Ta = Tb = Tc = 0.5f;
            break;
    }

    /* PWM占空比生成 */
    // 5. 占空比计算与限幅
    uint16_t pwm_a = (uint16_t)(Ta * PWM_Period);
    uint16_t pwm_b = (uint16_t)(Tb * PWM_Period);
    uint16_t pwm_c = (uint16_t)(Tc * PWM_Period);
//    // 6. PWM输出配置（需中央对齐模式）

	htim->Instance->CCR1 = pwm_a;
	htim->Instance->CCR2 = pwm_b;
	htim->Instance->CCR3 = pwm_c;



		static int i=0;
	i++;
}
