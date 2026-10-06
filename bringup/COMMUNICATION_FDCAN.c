/**
 * @file COMMUNICATION_FDCAN.c
 * @date 2026-10-06
 * @brief 早期 FDCAN 调速实现。现行电流环不走这里。
 *
 * 节拍限制见 COMMUNICATION_FDCAN.h 文件头。
 * @note 本头为后补。源文件更早，诞生日期以 git 为准。
 */

#include "COMMUNICATION_FDCAN.h"
#include "main.h"
#include "fdcan.h"
#include "stdint.h"
//motor_measure_t motor[7];
//	uint8_t rx_data[8];
// 鏂扮殑瀹忓畾涔夛細澶勭悊姣忎釜鐢垫満鐨勬暟鎹�锛堢姸鎬佸拰閫熷害锛�
the_new_order order;
#define deal_with_data(ptr, data, i) \
{ \
    uint16_t raw_value = (uint16_t)((data)[2*(i)] << 8 | (data)[2*(i)+1]); \
    uint8_t new_state = (uint8_t)((raw_value >> 15) & 0x01); \
    (ptr)->state = new_state; \
	motor_now.state = (ptr)->state; \
    /* 鏍规嵁鐘舵€佹洿鏂扮洰鏍囧€� */ \
    if (new_state == 0) { \
        /* 鐘舵€�0锛氭洿鏂扮數娴佸€硷紙mA鍗曚綅锛�*/ \
		(ptr)->given_current = raw_value & 0x7FFF; \
        motor_now.I_target = (ptr)->given_current; \
    } else { \
        /* 鐘舵€�1锛氭洿鏂伴€熷害鍊� */ \
		(ptr)->speed = raw_value & 0x7FFF;     \
       motor_now.speed_target = (ptr)->speed; \
    } \
}
FDCAN_TxHeaderTypeDef TxHeader;
FDCAN_RxHeaderTypeDef RxHeader;
extern uint8_t TxData[8];
extern uint8_t RxData[8];
/*------------------------ FDCAN 閰嶇疆鍑芥暟 ------------------------*/
void FDCAN1_Config(void)
{
    FDCAN_FilterTypeDef sFilterConfig;

    /*---------------- 1. 閰嶇疆鏍囧噯ID鎺╃爜婊ゆ尝鍣�锛堟帴鍙楁墍鏈夋爣鍑咺D锛� ----------------*/
    sFilterConfig.IdType       = FDCAN_STANDARD_ID;      // 鏍囧噯ID妯″紡
    sFilterConfig.FilterIndex  = 0;                     // 婊ゆ尝鍣ㄧ储寮�0
    sFilterConfig.FilterType   = FDCAN_FILTER_MASK;     // 鎺╃爜妯″紡
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; // 瀛樺叆RX FIFO0
    sFilterConfig.FilterID1    = 0x200;                // 鐩�鏍嘔D = 0x0000
    sFilterConfig.FilterID2    = 0x7FF;                // 鎺╃爜 = 0x0000锛堝叏涓嶆�€鏌ワ級

    HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig);
    /*---------------- 2. 鍏ㄥ眬杩囨护鍣ㄩ厤缃�锛堝叧閿�鎷掔粷鎵╁睍ID鍜岃繙绋嬪抚锛� ----------------*/
    HAL_FDCAN_ConfigGlobalFilter(
        &hfdcan1,
        FDCAN_REJECT, FDCAN_REJECT,
        FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE
    );
    /*---------------- 3. 鍚�鍔‵DCAN骞跺惎鐢ㄩ€氱煡 ----------------*/
    HAL_FDCAN_Start(&hfdcan1);
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    
    /*---------------- 4. 鍙戦€侀厤缃�锛堜繚鎸佷笌鍘烠AN涓€鑷达級 ----------------*/
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
    uint8_t rx_data[8]; // 鍥哄畾8瀛楄妭鏁版嵁锛堜笌鍘烠AN鍏煎�癸級
/*------------------------ FDCAN 鎺ユ敹鍥炶皟鍑芥暟 ------------------------*/
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{


    // 璇诲彇娑堟伅澶村拰鏁版嵁
     HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data); 

    /* 鍙岄噸楠岃瘉锛氱‘淇濇槸鏍囧噯ID鏁版嵁甯э紙闃插尽鎬х紪绋嬶級 */
    if ((rx_header.IdType == FDCAN_STANDARD_ID) && 
        (rx_header.RxFrameType == FDCAN_DATA_FRAME)) 
    {
                // 璁＄畻鐢垫満绱㈠紩锛堜笌鍘熼€昏緫瀹屽叏涓€鑷达級
//                uint8_t i = TxHeader.Identifier - CAN1_M1_ID;
// deal_with_data(&order, rx_data, i); //
    }
}
uint8_t TxData[8];
void FDCAN_MOTER_START(FDCAN_HandleTypeDef *hfdcan, 
	int16_t ecd, int16_t speed_rpm, int16_t given_current,int16_t temperate)
{

    // 鍑嗗�囨秷鎭�鏁版嵁
// 澶х��妯″紡锛堥珮8浣嶅湪鍓嶏級锛屼娇鐢ㄤ綅鎺╃爜闃叉�㈢�﹀彿鎵╁睍闂�棰�
TxData[0] = (ecd >> 8);  // M1楂�8浣�
TxData[1] = ecd;        // M1浣�8浣�
TxData[2] = (speed_rpm >> 8);  // M2楂�8浣�
TxData[3] = speed_rpm;         // M2浣�8浣�
TxData[4] = (given_current >> 8);  // M3楂�8浣�
TxData[5] = given_current;         // M3浣�8浣�
TxData[6] = (temperate >> 8);  // M4楂�8浣�
TxData[7] = temperate;         // M4浣�8浣�

    // 鍙戦€佹秷鎭�
    HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxHeader, TxData);
}

