#ifndef __AS5047_H
#define __AS5047_H

#include "spi.h"

// AS5047p 地址
#define NOP 0x0000
#define ERRFL 0x0001
#define PROG 0x0003
#define DIAAGC 0x3FFC
#define MAG 0x3FFD
#define ANGLEUNC 0x3FFE
#define ANGLECOM 0x3FFF

#define ZPOSM 0x0016
#define ZPOSL 0x0017
#define SETTINGS1 0x0018
#define SETTINGS2 0x0019
#define AS5047_RESOLUTION 16384 //12bit Resolution 

// AS5047实例结构体
typedef struct {
    SPI_HandleTypeDef *hspi;           // SPI句柄
    GPIO_TypeDef *cs_gpio_port;        // CS引脚端口
    uint16_t cs_gpio_pin;              // CS引脚
    float angle_data_prev;             // 上次位置
    float full_rotation_offset;        // 转过的整圈数
} AS5047_HandleTypeDef;
extern AS5047_HandleTypeDef AS5047_spi1_PORT, AS5047_spi3_PORT;
// CS引脚控制宏（需要根据实际硬件修改引脚定义）
#define AS5047_CS_L(handle) HAL_GPIO_WritePin(handle->cs_gpio_port, handle->cs_gpio_pin, GPIO_PIN_RESET)
#define AS5047_CS_H(handle) HAL_GPIO_WritePin(handle->cs_gpio_port, handle->cs_gpio_pin, GPIO_PIN_SET)

// 函数声明
uint16_t Parity_bit_Calculate(uint16_t data_2_cal);

uint16_t SPI_ReadWrite_OneByte(AS5047_HandleTypeDef *as5047, uint16_t _txdata);
uint16_t AS5047_read(AS5047_HandleTypeDef *as5047, uint16_t add);
float AS5047_GetAngle(AS5047_HandleTypeDef *as5047);
void AS5047_Init(AS5047_HandleTypeDef *as5047, SPI_HandleTypeDef *hspi, 
                 GPIO_TypeDef *cs_port, uint16_t cs_pin);

#endif