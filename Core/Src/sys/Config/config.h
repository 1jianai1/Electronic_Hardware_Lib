//
// Created by jianai on 2025/4/23.
//

#ifndef TEST_CONFIG_H
#define TEST_CONFIG_H


#include "main.h"

#include <stdio.h>
#include <stdbool.h>

#include "../delay/delay.h"

// todo 方便开发的宏定义
typedef uint8_t  u8;
typedef uint16_t u16;
typedef uint32_t u32;

#define FALSE   0
#define TRUE    1

typedef enum{
    DL_OK,		// 完成
    DL_ERROR,	// 错误
    DL_WAIT		// 等待
}DEBUGSTATE;

// todo GPIO口状态
typedef enum{
    LOW,		// 低
    HIGH,		// 高
    TURN		// 转变
}GPIOLEVEL;


// GPIO 状态
void digitalWrite(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIOLEVEL LEVEL);
uint8_t digitalRead(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);

// todo 全局参数初始化
#define RX1_LEN  1

typedef struct {
    //串口接收数据标志位
    uint8_t rx1_flag;
    uint8_t rx1_len;
    uint8_t rx1_buff[RX1_LEN];
}CONFIG;


extern CONFIG config;
void config_init(void);


#endif //TEST_CONFIG_H
