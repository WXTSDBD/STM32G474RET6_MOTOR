#ifndef _FOC_CAL_H
#define _FOC_CAL_H
#include "tim.h"
#include "Sliding_Window_Filter.h"
typedef struct 
{
    float get;
    float use;
    int raw;
    float add;
    float last;
    float target;
    float sin_val;
    float cos_val;
    float mech,gap,stop;
    float rpm_value,rpm_fliter;
    // === 新增速度计算相关变量 ===
    uint32_t last_time_us;      // 上次时间戳(微秒)
    float speed_rad_s;          // 角速度(rad/s)
    float speed_filtered;       // 滤波后的速度
    uint8_t speed_ready;        // 速度数据就绪标志
} angle;

extern float rpm_filtered; 

typedef struct
{
    float angle_get;
	float angle_use;
//	DATA_ADC foc;
}foc_station;


//extern foc trans;

#define LOW_DISABLE 0
#define LOW_ENABLE 1
#define STATE_PID 0
#define STATE_BRAKE 1
extern float Uq_ramp, Ud_ramp ; // d轴和q轴电压（斜坡处理后）
extern float Ud , Uq; // d轴和q轴电压（未处理）
typedef struct
{ 
	int IA_OFF;
	int IB_OFF;
	int IC_OFF;

}ADC_OFFSET;
typedef struct
{ 
	float lpf_IA,lpf_IB,lpf_IC,lpf_IBUS,lpf_DBUS;
}ADC_FILITER;
typedef struct
{
    float IA,IB,IC,IDCBUS,VDCBUS;

}ADC_USE;
//extern float rpm_filtered;

//extern float UQ1;
//extern float *UQ_PRT;
//extern float UD1;
//extern float *UD_PRT;

extern angle as5047_spi1;
extern angle as5047_spi3;
float _normalizeAngle(float angle);
//void setPhaseVoltage(float Uq, float Ud, float angle_el);
void AS5047_Calculate_Speed(angle *enc, MovingAverageFilter *filter);
void VF_OPENLOOP_TASK(TIM_HandleTypeDef *htim,angle *angle_now);
void angle_init(void);

#endif


