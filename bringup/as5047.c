#include "tim.h"
#include "gpio.h"
#include "stdio.h"
#include "spi.h"
#include "as5047.h"

#define abs(x) ((x)>0?(x):-(x))
#define _2PI 6.28318530718f
AS5047_HandleTypeDef AS5047_spi1_PORT, AS5047_spi3_PORT;
// 计算奇偶校验位
uint16_t Parity_bit_Calculate(uint16_t data_2_cal)
{
    uint16_t parity_bit_value = 0;
    while(data_2_cal != 0)
    {
        parity_bit_value ^= data_2_cal; 
        data_2_cal >>= 1;
    }
    return (parity_bit_value & 0x1); 
}

// SPI发送读取函数（支持多个SPI实例）
uint16_t SPI_ReadWrite_OneByte(AS5047_HandleTypeDef *as5047, uint16_t _txdata)
{
    AS5047_CS_L(as5047);  // CS拉低
    
    uint16_t rxdata;
    if(HAL_SPI_TransmitReceive(as5047->hspi, (uint8_t *)&_txdata, 
                              (uint8_t *)&rxdata, 1,1000) != HAL_OK) {
        rxdata = 0;  // 通信失败返回0
    }
    
    AS5047_CS_H(as5047);  // CS拉高
    return rxdata;
}

// AS5047读取函数
uint16_t AS5047_read(AS5047_HandleTypeDef *as5047, uint16_t add)
{
    uint16_t data;
    add |= 0x4000;  // 读指令 bit14 置1
    
    // 如果前15位1的个数为偶数，则Bit15置1
    if(Parity_bit_Calculate(add) == 1) {
        add = add | 0x8000;
    }
    
    SPI_ReadWrite_OneByte(as5047, add);  // 发送指令
    data = SPI_ReadWrite_OneByte(as5047, NOP | 0x4000);  // 发送空指令读取数据
    
    data &= 0x3fff;  // 取14位数据
    return data;
}

// 获取角度（弧度）
float AS5047_GetAngle(AS5047_HandleTypeDef *as5047)
{
    float angle_data = AS5047_read(as5047, ANGLEUNC);
    
    // 处理角度翻转
    float d_angle = angle_data - as5047->angle_data_prev;
    if(abs(d_angle) > (0.8f * AS5047_RESOLUTION)) {
        as5047->full_rotation_offset += (d_angle > 0 ? -_2PI : _2PI);
    }
    as5047->angle_data_prev = angle_data;
    
    return (as5047->full_rotation_offset + (angle_data / (float)AS5047_RESOLUTION) * _2PI);
}

// AS5047初始化函数
void AS5047_Init(AS5047_HandleTypeDef *as5047, SPI_HandleTypeDef *hspi, 
                 GPIO_TypeDef *cs_port, uint16_t cs_pin)
{
    as5047->hspi = hspi;
    as5047->cs_gpio_port = cs_port;
    as5047->cs_gpio_pin = cs_pin;
    as5047->angle_data_prev = 0;
    as5047->full_rotation_offset = 0;
    
    // 初始化CS引脚为高电平
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
}