#include "COMMUNICATION_FDCAN.h"
#include "main.h"
#include "fdcan.h"
#include "stdint.h"
//motor_measure_t motor[7];
//	uint8_t rx_data[8];
// 新的宏定义：处理每个电机的数据（状态和速度）
the_new_order order;
#define deal_with_data(ptr, data, i) \
{ \
    uint16_t raw_value = (uint16_t)((data)[2*(i)] << 8 | (data)[2*(i)+1]); \
    uint8_t new_state = (uint8_t)((raw_value >> 15) & 0x01); \
    (ptr)->state = new_state; \
	motor_now.state = (ptr)->state; \
    /* 根据状态更新目标值 */ \
    if (new_state == 0) { \
        /* 状态0：更新电流值（mA单位）*/ \
		(ptr)->given_current = raw_value & 0x7FFF; \
        motor_now.I_target = (ptr)->given_current; \
    } else { \
        /* 状态1：更新速度值 */ \
		(ptr)->speed = raw_value & 0x7FFF;     \
       motor_now.speed_target = (ptr)->speed; \
    } \
}
FDCAN_TxHeaderTypeDef TxHeader;
FDCAN_RxHeaderTypeDef RxHeader;
extern uint8_t TxData[8];
extern uint8_t RxData[8];
/*------------------------ FDCAN 配置函数 ------------------------*/
void FDCAN1_Config(void)
{
    FDCAN_FilterTypeDef sFilterConfig;

    /*---------------- 1. 配置标准ID掩码滤波器（接受所有标准ID） ----------------*/
    sFilterConfig.IdType       = FDCAN_STANDARD_ID;      // 标准ID模式
    sFilterConfig.FilterIndex  = 0;                     // 滤波器索引0
    sFilterConfig.FilterType   = FDCAN_FILTER_MASK;     // 掩码模式
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; // 存入RX FIFO0
    sFilterConfig.FilterID1    = 0x200;                // 目标ID = 0x0000
    sFilterConfig.FilterID2    = 0x7FF;                // 掩码 = 0x0000（全不检查）

    HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig);
    /*---------------- 2. 全局过滤器配置（关键拒绝扩展ID和远程帧） ----------------*/
    HAL_FDCAN_ConfigGlobalFilter(
        &hfdcan1,
        FDCAN_REJECT, FDCAN_REJECT,
        FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE
    );
    /*---------------- 3. 启动FDCAN并启用通知 ----------------*/
    HAL_FDCAN_Start(&hfdcan1);
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    
    /*---------------- 4. 发送配置（保持与原CAN一致） ----------------*/
    TxHeader.Identifier           = CAN1_M2_ID;
    TxHeader.IdType               = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType          = FDCAN_DATA_FRAME;
    TxHeader.DataLength           = FDCAN_DLC_BYTES_8;
    TxHeader.ErrorStateIndicator  = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch        = FDCAN_BRS_OFF;
    TxHeader.FDFormat             = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl   = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker        = 0x00;
}
    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8]; // 固定8字节数据（与原CAN兼容）
/*------------------------ FDCAN 接收回调函数 ------------------------*/
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{


    // 读取消息头和数据
     HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data); 

    /* 双重验证：确保是标准ID数据帧（防御性编程） */
    if ((rx_header.IdType == FDCAN_STANDARD_ID) && 
        (rx_header.RxFrameType == FDCAN_DATA_FRAME)) 
    {
                // 计算电机索引（与原逻辑完全一致）
//                uint8_t i = TxHeader.Identifier - CAN1_M1_ID;
// deal_with_data(&order, rx_data, i); //
    }
}
uint8_t TxData[8];
void FDCAN_MOTER_START(FDCAN_HandleTypeDef *hfdcan, 
	int16_t ecd, int16_t speed_rpm, int16_t given_current,int16_t temperate)
{

    // 准备消息数据
// 大端模式（高8位在前），使用位掩码防止符号扩展问题
TxData[0] = (ecd >> 8);  // M1高8位
TxData[1] = ecd;        // M1低8位
TxData[2] = (speed_rpm >> 8);  // M2高8位
TxData[3] = speed_rpm;         // M2低8位
TxData[4] = (given_current >> 8);  // M3高8位
TxData[5] = given_current;         // M3低8位
TxData[6] = (temperate >> 8);  // M4高8位
TxData[7] = temperate;         // M4低8位

    // 发送消息
    HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxHeader, TxData);
}

