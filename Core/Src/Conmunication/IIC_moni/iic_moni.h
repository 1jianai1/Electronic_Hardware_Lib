//
// Created by jianai on 2025/4/24.
//

#ifndef TEST_IIC_MONI_H
#define TEST_IIC_MONI_H

#include "main.h"

//#define MPU_HARDWARE

#ifdef  MPU_HARDWARE
#include "i2c.h"

#else

#endif


typedef enum{
    SOFTWARE,
    HARDWARE
}IIC_MODE;

typedef enum{
    QMC5883_IIC,
    OLED_IIC,
    MPU6050_IIC,
    IIC_SUM
}IIC_ID;

typedef struct{
    GPIO_TypeDef* SCL_PORT;
    uint16_t SCL;
    GPIO_TypeDef* SDA_PORT;
    uint16_t SDA;

    IIC_MODE mode;   // todo 硬件 or 软件标志位

    void (*W_SDA)(IIC_ID, uint8_t);
    void (*W_SCL)(IIC_ID, uint8_t);
    uint8_t (*R_SDA)(IIC_ID);
    uint8_t (*ReadReg)(IIC_ID, uint8_t, uint8_t, uint8_t*, uint16_t);		//IIC读取寄存器接口
    uint8_t (*WriteReg)(IIC_ID, uint8_t, uint8_t, uint8_t*, uint16_t);	//IIC写入寄存器接口
}IICINTERFACE;  // IIC调用接口函数

extern IICINTERFACE iic[IIC_SUM];

void iic_attach(
        IIC_ID id,
        GPIO_TypeDef* SCL_PORT,
        uint16_t SCL,
        GPIO_TypeDef* SDA_PORT,
        uint16_t SDA,
        IIC_MODE mode);

uint8_t I2C_WriteDatas(IIC_ID id, uint8_t addr, uint8_t reg, uint8_t *pdata, uint16_t pNum);
uint8_t I2C_ReadDatas(IIC_ID id, uint8_t addr, uint8_t reg, uint8_t *pdata, uint16_t pNum);
#endif //TEST_IIC_MONI_H
