#ifndef __AS5047_H
#define __AS5047_H

#include "spi.h"

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
#define AS5047_RESOLUTION 16384

typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *cs_gpio_port;
    uint16_t cs_gpio_pin;
    float angle_data_prev;
    float full_rotation_offset;
} AS5047_HandleTypeDef;
extern AS5047_HandleTypeDef AS5047_spi1_PORT, AS5047_spi3_PORT;

#define AS5047_CS_L(handle) \
    ((handle)->cs_gpio_port->BSRR = ((uint32_t)((handle)->cs_gpio_pin) << 16U))
#define AS5047_CS_H(handle) \
    ((handle)->cs_gpio_port->BSRR = (uint32_t)((handle)->cs_gpio_pin))

uint16_t Parity_bit_Calculate(uint16_t data_2_cal);

uint16_t SPI_ReadWrite_OneByte(AS5047_HandleTypeDef *as5047, uint16_t _txdata);
uint16_t AS5047_read(AS5047_HandleTypeDef *as5047, uint16_t add);
float AS5047_GetAngle(AS5047_HandleTypeDef *as5047);
void AS5047_Init(AS5047_HandleTypeDef *as5047, SPI_HandleTypeDef *hspi,
                 GPIO_TypeDef *cs_port, uint16_t cs_pin);

/* SPI1 LL DMA: TIM1 DmaKick + GetRaw @20kHz (see g_telem_dbg.enc_*) */
void AS5047_DmaInit(AS5047_HandleTypeDef *as5047);
void AS5047_DmaKick(AS5047_HandleTypeDef *as5047);
uint16_t AS5047_GetRaw(void);

void AS5047_Spi1LL_OnRxComplete(void);
void AS5047_Spi1LL_OnError(void);

#endif /* __AS5047_H */
