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
