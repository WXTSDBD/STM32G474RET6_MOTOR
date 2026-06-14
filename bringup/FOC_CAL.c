#include "FOC_CAL.h"
#include <math.h> 
#include "gpio.h"       // ����GPIO��ͨ��������������ܵ�ͷ�ļ�
#include "arm_math.h"   // ����ARM��ѧ���ͷ�ļ��������Ż���ѧ����
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
int elect_pair_num = 14;//�缫������2312s�缫����Ϊ12��tmotor_u3,7
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

// �������
#define MOTOR_KV  220.0f  // ���KVֵ��RPM/V��
#define MOTOR_KE  (60.0f / (2.0f * _PI * MOTOR_KV)) // ���綯�Ƴ�����V��s/rad��
#define MOTOR_KT  MOTOR_KE // ת�س�����Nm/A��

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
* ���ܣ��޸п�����������
* ʵ���߼���
* 1. ��ȡ������ԭʼ�Ƕ�����
* 2. ��ȡ������ĽǶ�ֵ
* 3. ���㲢���µ�Ƕ�
* 4. �������ѹ
* 5. ���㵱ǰ�ٶ�
* 6. �����ϴνǶ�ֵ
*****************************************************************************/
void VF_OPENLOOP_TASK(TIM_HandleTypeDef *htim,angle *angle_now)
{
    // �������ѹ
    setPhaseVoltage(htim, uq_intput,0, angle_el); // �������ѹ��ud_inputΪ�����ѹ��angle_elΪ��Ƕ�

    // ���µ�Ƕȹ���ֵ
    angle_el = angle_el + angle_now->add; // �Ե�ǶȽ��в�����У׼
}
