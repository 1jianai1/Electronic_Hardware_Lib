//
// Created by jianai on 2024/11/12.
//

#ifndef STM32_CONTROL_MOTOR_H
#define STM32_CONTROL_MOTOR_H

#include "main.h"
#include "tim.h"

//todo 你用的是什么芯片
/*
 * TB6612/L298N: IN1 和 IN2 的高低电平驱动 IN1 高 IN2 低 为一个方向，IN1 低 IN2 高 为另一个方向
 * DRV8701 : IN1 时钟为低（两路电机可以共阴）  IN2 高低控制正反转
 * */
//#define TB6612
#define DRV8701

// 对应电机编号
typedef enum{
    M1,
    M_SUM
}MotorID;

typedef void (*VOID)(void);
typedef void (*MOTOR_INTERFACE)(MotorID);
typedef void (*MOTOR_SPEED_INTERFACE)(MotorID, int16_t);

// 可以采用数组索引的方式进行封装
//电机类的属性
typedef struct {
    //接口属性
    //方向接口
    GPIO_TypeDef* GPIOx_1;
    GPIO_TypeDef* GPIOx_2;
    uint16_t M_IN1;
    uint16_t M_IN2;
    //定时器接口
    uint16_t arr;
    uint16_t psc;
    TIM_HandleTypeDef htim;  //定时器句柄
    uint32_t tim_channel;    //CH通道
    TIM_HandleTypeDef entim;
}MOTOR_HARDWARE;

// 对外接口 - 硬件层用hw访问
typedef struct {
    MOTOR_HARDWARE* hw;  // 硬件接口

    //运动属性
    int en_actual;//实际速度 -- 编码器测得

    // PID控制属性
    int tar_speed;//目标速度 -- 直接设置
    int pre_tar_speed;     // 用于前馈  上一次目标
    float err;          // 误差
    float pre_err;      // 上一次误差
    float integral;     // 积分值
    float output;       // pid输出值

    // 基本函数接口
    MOTOR_INTERFACE positive;  // 方向正转
    MOTOR_INTERFACE nagitive;  // 方向反转
    MOTOR_INTERFACE stop;       // 停止
    MOTOR_INTERFACE getSpeed;  // 获取速度
    MOTOR_SPEED_INTERFACE  setSpeed; // 速度赋值

    // PID控制接口
    MOTOR_SPEED_INTERFACE   setTarSpeed;    // PID控制 -- 设置目标速度
    MOTOR_INTERFACE pidSpeedloop;  // 单速度环控速接口 --- 此函数应该在定时器中断中被定时调用

}MOTOR;

extern MOTOR motor[M_SUM];

//电机初始化
void motor_attach(
        MotorID id,
        TIM_HandleTypeDef htim,  //定时器句柄
        uint32_t tim_channel,    //CH通道
        TIM_HandleTypeDef entim,
        GPIO_TypeDef* GPIOx_1,
        uint16_t M_IN1,
        GPIO_TypeDef* GPIOx_2,
        uint16_t M_IN2
);
void motor_init(void);
#endif //STM32_CONTROL_MOTOR_H
