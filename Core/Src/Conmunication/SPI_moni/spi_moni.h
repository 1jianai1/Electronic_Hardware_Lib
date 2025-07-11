//
// Created by jianai on 2025/6/11.
//

#ifndef TEST_SPI_MONI_H
#define TEST_SPI_MONI_H
#include "main.h"

typedef enum{
    FULL,
    ONLY_SEND,
    ONLY_RECEIVE,
    HARDWARE
}SPI_MODE;

typedef enum{
    OLED_SPI,
    IMU66RA_SPI,
    SPI_SUM
}SPI_ID;


typedef struct{
    GPIO_TypeDef* SCL_PORT;
    uint16_t SCL;               // 时钟
    GPIO_TypeDef* MOSI_PORT;
    uint16_t MOSI;              // 发送
    GPIO_TypeDef* MISO_PORT;
    uint16_t MISO;              // 接收
    GPIO_TypeDef* CE_PORT;
    uint16_t CE;                //  片选


    SPI_MODE mode;   // todo 硬件 or 软件标志位

    void (*W_SCL)(SPI_ID, uint8_t);
    void (*W_SS)(SPI_ID, uint8_t);
    void (*W_MOSI)(SPI_ID, uint8_t);
    uint8_t (*R_MISO)(SPI_ID);
    uint8_t (*SwapByte)(SPI_ID, uint8_t);
    uint8_t (*SwapDatas)(SPI_ID, uint8_t*, uint16_t);
    uint8_t (*SendDatas)(SPI_ID, uint8_t*, uint16_t);
    uint8_t (*ReceiveDatas)(SPI_ID, uint8_t*, uint16_t);

}SPIINTERFACE;  // IIC调用接口函数

extern SPIINTERFACE spi[SPI_SUM];

uint8_t spi_attach(
        SPI_ID id,
        GPIO_TypeDef* SCL_PORT,
        uint16_t SCL,               // 时钟
        GPIO_TypeDef* MOSI_PORT,
        uint16_t MOSI,              // 发送
        GPIO_TypeDef* MISO_PORT,
        uint16_t MISO,              // 接收
        GPIO_TypeDef* CE_PORT,
        uint16_t CE,                //  片选
        SPI_MODE mode
);

#endif //TEST_SPI_MONI_H
