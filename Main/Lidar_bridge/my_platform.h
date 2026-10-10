#include "stm32f4xx.h"
#include "vl53l0x_i2c_platform.h"
#include "vl53l0x_api.h"

#ifndef MY_PLAT_H
#define MY_PLAT_H
extern volatile uint32_t ms_ticks;
int32_t I2C_write_multi(uint8_t dev_addr,
                     uint8_t reg,
                     uint8_t *pdata,
                     uint32_t count);



int32_t I2C_read_multi(uint8_t dev_addr,
                       uint8_t reg,
                       uint8_t *pdata,
                       uint32_t count);


void init_func(void);
void SysTick_Init(void);
void delay_ms(uint32_t ms);
void MPU_Read (uint8_t Address, uint8_t Reg, uint8_t *buffer, uint8_t size);

void MPU_Write_Multiple(uint8_t Address,
                        uint8_t Reg,
                        uint8_t *data,
                        uint8_t size);


extern VL53L0X_Dev_t dev;
#endif