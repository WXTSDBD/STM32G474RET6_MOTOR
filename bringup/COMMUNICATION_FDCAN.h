/**
 * @file COMMUNICATION_FDCAN.h
 * @date 2026-10-06
 * @brief 早期 FDCAN 调速口。现行电流环不走这里。

 *
 * 保留给旧上位机。新业务不要再往本头加接口。
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
 */

#ifndef __COMMUNICATION_FDCAN_H__
#define __COMMUNICATION_FDCAN_H__
#include "stdint.h"
#include "fdcan.h"
typedef struct
{
    uint8_t state;
	uint16_t speed;
	uint16_t given_current;
}the_new_order;
extern the_new_order order;
extern uint8_t rx_data[8]; 
typedef enum
{
	
    CAN_ALL_ID = 0x200,
    CAN1_M1_ID = 0x201,
	CAN1_M2_ID = 0x202,
	CAN1_M3_ID = 0x203,
	CAN1_M4_ID = 0x204,
	CAN2_M5_ID = 0x205,
	CAN2_M6_ID = 0x206,
	CAN2_M7_ID = 0x207,
    CAN2_ALL_ID = 0x1FF,
}CAN_MOTOR;

void FDCAN1_Config(void);
void FDCAN_MOTER_START(FDCAN_HandleTypeDef *hfdcan, int16_t ecd, int16_t speed_rpm,
	int16_t given_current, int16_t temperate);
#endif
