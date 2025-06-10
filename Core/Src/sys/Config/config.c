//
// Created by jianai on 2025/4/23.
//
#include "config.h"

uint8_t digitalRead(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin){
    GPIO_PinState bitstatus;
    assert_param(IS_GPIO_PIN(GPIO_Pin));

    if((GPIOx->IDR & GPIO_Pin) != (uint32_t)GPIO_PIN_RESET)
    {
        bitstatus = GPIO_PIN_SET;
    }
    else
    {
        bitstatus = GPIO_PIN_RESET;
    }
    return bitstatus;
}

void digitalWrite(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIOLEVEL LEVEL){
    assert_param(IS_GPIO_PIN(GPIO_Pin));
    assert_param(IS_GPIO_PIN_ACTION(PinState));
    u32 odr;
    switch(LEVEL){
        case LOW:
            GPIOx->BSRR =  (uint32_t)GPIO_Pin << 16U;
            break;
        case HIGH:
            GPIOx->BSRR = GPIO_Pin;
            break;
        case TURN:
            odr = GPIOx->ODR;
            GPIOx->BSRR = ((odr & GPIO_Pin) << 16U) | (~odr & GPIO_Pin);
            break;
    }
}


CONFIG config;
void config_init(void){
    config.rx1_len = 1;
    config.rx1_flag = 0;
}


int __io_putchar(int ch)
{
    while ((USART1->SR & 0X40) == 0); // 等�
    // ��上一次发送完
    USART1->DR = (uint8_t)ch; //串口发�?�字�??????????????????????????????
    return 1;
}