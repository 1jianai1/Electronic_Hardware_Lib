#ifndef __OLED_H
#define __OLED_H

#include "main.h"
//todo 硬件配置
#define OLED_MAX_W 128
#define OLED_MAX_H 64
#define OLED_MAX_P 8

#define I2C_HARDWARE 0  //todo 是否使用硬件I2C:1 使用 2 不适用

#if I2C_HARDWARE==1//硬件I2C
#include "i2c.h"
extern I2C_HandleTypeDef hi2c1;

#define OLED_I2C hi2c1
#define OLED_ADDRESS 0x78//？？？？是吗

#else//软件I2C

//SCL
#define OLED_I2C_SCL_Port     GPIOB
#define OLED_I2C_SCL_Pin      GPIO_PIN_8
//SDA
#define OLED_I2C_SDA_Port     GPIOB
#define OLED_I2C_SDA_Pin      GPIO_PIN_9
#endif




void OLED_Init(void);
void OLED_Clear(void);
void OLED_SetCursor(uint8_t Y, uint8_t X);
void OLED_WriteData(uint8_t Data);
void OLED_ShowChar(uint8_t Line, uint8_t Column, char Char);
void OLED_ShowString(uint8_t Line, uint8_t Column, char *String);
void OLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
void OLED_ShowSignedNum(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length);
void OLED_ShowHexNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
void OLED_ShowBinNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
void OLED_ShowFloat(uint8_t Line,uint8_t Column,float Number,uint8_t Length,uint8_t point_len);

#endif
