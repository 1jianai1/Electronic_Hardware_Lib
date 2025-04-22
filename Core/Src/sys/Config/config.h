//
// Created by jianai on 2025/4/23.
//

#ifndef TEST_CONFIG_H
#define TEST_CONFIG_H

#include "main.h"

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
