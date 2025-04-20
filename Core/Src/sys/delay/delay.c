//
// Created by jianai on 2025/4/19.
//

#include "delay.h"

void delay_us(uint32_t udelay)
{
    uint32_t startval,tickn,delays,wait;

    startval = SysTick->VAL;
    tickn = HAL_GetTick();
    //sysc = 72000;  //SystemCoreClock / (1000U / uwTickFreq);
    delays =udelay * MAIN_FREQ; //sysc / 1000 * udelay;
    if(delays > startval)
    {
        while(HAL_GetTick() == tickn)
        {

        }
        wait = MAIN_FREQ * 1000 + startval - delays;
        while(wait < SysTick->VAL)
        {

        }
    }
    else
    {
        wait = startval - delays;
        while(wait < SysTick->VAL && HAL_GetTick() == tickn)
        {

        }
    }
}
