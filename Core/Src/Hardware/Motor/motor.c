//
// Created by jianai on 2024/11/12.
//
#include "motor.h"
#include "gpio.h"
#include "tim.h"
#include <stdio.h>


//电机硬件宏参数赋值
MOTOR_HARDWARE motorhardware[M_SUM]={
        {
                GPIOB,
                GPIOB,
                GPIO_PIN_15,
                GPIO_PIN_14,
                0,
                0,
                {0},
                TIM_CHANNEL_1,
                {0}
        }
};
MOTOR motor[M_SUM] = {
        {
                &motorhardware[M1],  // 重定义硬件接口, 直接在上面声明赋值结构体即可
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                NULL,
                NULL,
                NULL,
                NULL,
                NULL,
                NULL,
                NULL
        }
};
//电机方向单独控制--这里仅针对两个芯片进行了封装 --- TB6612 and DRV8701
static void M_P(MotorID id){//电机正转
#ifdef TB6612
    motor[id].hw->GPIOx_1->BSRR = motor[id].hw->M_IN1;
    motor[id].hw->GPIOx_2->BSRR = (uint32_t)motor[id].hw->M_IN2<<16U;
#endif
#ifdef DRV8701
    motor[id].hw->GPIOx_1->BSRR = (uint32_t )motor[id].hw->M_IN1<<16U;
    motor[id].hw->GPIOx_2->BSRR = (uint32_t)motor[id].hw->M_IN2<<16U;
#endif
}
static void M_N(MotorID id){//电机1反转
#ifdef TB6612
    motor[id].hw->GPIOx_1->BSRR = (uint32_t)motor[id].hw->M_IN1<<16U;
    motor[id].hw->GPIOx_2->BSRR = motor[id].hw->M_IN2;
#endif
#ifdef DRV8701
    motor[id].hw->GPIOx_1->BSRR = (uint32_t)motor[id].hw->M_IN1<<16U;
    motor[id].hw->GPIOx_2->BSRR = (uint32_t)motor[id].hw->M_IN2;
#endif
}
static void M_S(MotorID id) {//电机停止
#ifdef TB6612
    motor[id].hw->GPIOx_1->BSRR = (uint32_t) motor[id].hw->M_IN1 << 16U;
    motor[id].hw->GPIOx_2->BSRR = (uint32_t) motor[id].hw->M_IN2 << 16U;
#endif
#ifdef DRV8701
   motor[id].setSpeed(id, 0);
#endif
}

static void set_speed(MotorID id,int16_t speed){//速度单独设置
        if(speed>=0) {
            M_P(id);
            __HAL_TIM_SET_COMPARE(&motor[id].hw->htim, motor[id].hw->tim_channel, speed);
        }
        else{
            M_N(id);
            __HAL_TIM_SET_COMPARE(&motor[id].hw->htim,motor[id].hw->tim_channel,-speed);
        }
}

static void get_speed(MotorID id){
    motor[id].en_actual = (short)__HAL_TIM_GET_COUNTER(&motor[id].hw->entim);
    //编码器寄存器置0
    __HAL_TIM_SET_COUNTER(&motor[id].hw->entim, 0);
}

// 结构体引脚定向
void motor_attach(
        MotorID id,
        TIM_HandleTypeDef htim,  //定时器句柄
        uint32_t tim_channel,    //CH通道
        TIM_HandleTypeDef entim,
        GPIO_TypeDef* GPIOx_1,
        uint16_t M_IN1,
        GPIO_TypeDef* GPIOx_2,
        uint16_t M_IN2
){
    MOTOR* id_motor = &motor[id];
    id_motor->hw->htim = htim;                  // 定时器定向
    id_motor->hw->tim_channel = tim_channel;    // CH通道定向
    id_motor->hw->entim = entim;                // 编码器
    id_motor->hw->GPIOx_1 = GPIOx_1;
    id_motor->hw->GPIOx_2 = GPIOx_2;            // 方向通道定向
    id_motor->hw->M_IN1 = M_IN1;
    id_motor->hw->M_IN2 = M_IN2;

    // 函数指针重定向  - 你当然也可以自定义函数指针添加进来 前提是得满足声明要求
    id_motor->positive = M_P;
    id_motor->nagitive = M_N;
    id_motor->stop = M_S;
    id_motor->setSpeed = set_speed;
    id_motor->getSpeed = get_speed;

}

// 硬件初始化
void motor_init(void){
    for(uint8_t i=0; i < M_SUM;i++){
        // 获取相关参数
        motor[i].hw->psc = motor[i].hw->htim.Instance->PSC;
        motor[i].hw->arr = motor[i].hw->htim.Instance->ARR;

        //启动 PWM + 编码器定时器
        HAL_TIM_PWM_Start(&motor[i].hw->htim,motor[i].hw->tim_channel);
        HAL_TIM_Encoder_Start(&motor[i].hw->entim,TIM_CHANNEL_ALL);
    }
}






