//
// Created by jianai on 2025/4/21.
//
#ifndef __STEP_MOTOR_H__
#define __STEP_MOTOR_H__

#include "main.h"
#include "tim.h"
#define   Pulse      800         //4ϸ��ʱ�����Ϊ0.45�� 800������תһȦ �����������1.8�ȣ���8ϸ��ʱ�����Ϊ0.225��1600������תһȦ
#define   Hclk       1000000  //时间总频

//todo 你用的是什么芯片
/*
 * */
#define TB6600

// 编号
typedef enum {
    SM1,
    STEP_SUM
}STEPID;

// todo ------------------- 类型宏定义 --------------------
typedef void (*VOID)(void);
typedef void (*STEPMOTOR_INTERFACE)(STEPID);
typedef void (*STEPMOTOR_RANGE_INTERFACE)(STEPID, char, int32_t, int32_t);
typedef void (*STEPMOTOR_SPEED_INTERFACE)(STEPID, int32_t);
typedef void (*STEPMOTOR_STEP_INTERFACE)(STEPID, int32_t, uint32_t);

typedef struct {
    // todo 硬件参数层
    TIM_HandleTypeDef* htim;    //定时器句柄
    uint32_t Channel;           // 输出通道
    GPIO_TypeDef* enType;       //使能引脚类别
    uint16_t  enPin;            //使能引脚pin
    GPIO_TypeDef* dirType;      //方向引脚类别
    uint16_t  dirPin;           //方向引脚pin

    // todo 硬件API重写层
    STEPMOTOR_INTERFACE enSet;                // 方向引脚置位
    STEPMOTOR_INTERFACE enReset;                // 方向引脚置位
    STEPMOTOR_INTERFACE dirSet;                // 方向引脚置位
    STEPMOTOR_INTERFACE dirReset;                // 方向引脚置位
    STEPMOTOR_INTERFACE htimStop;              // 停止并复位定时器
    STEPMOTOR_INTERFACE htimStart;              // 重新启动定时器

    STEPMOTOR_INTERFACE pwmStop;                // pwm停止
    STEPMOTOR_INTERFACE pwmStart;               // pwm启动
    STEPMOTOR_INTERFACE pwmStopIT;             // pwm中断计数停止
    STEPMOTOR_INTERFACE pwmStartIT;            // pwm中断计数启动
    STEPMOTOR_SPEED_INTERFACE pwmSetCompare;          // pwm设置为50%的占空比

}STEPMOTOR_HARDWARE;

typedef struct{

    STEPMOTOR_HARDWARE* hw;  // 硬件接口封装层

    // todo 参数层
    uint32_t cur_freq;  // 当前频率Hz
    uint16_t cur_step;  // 当前步数
    uint8_t dir;        // 当前方向

    // todo 目标值
    uint16_t tar_step;  // 目标步数

    // todo 限制层
    uint8_t is_limit_step;          // 你是否要限制步数
    uint8_t is_finish;              // 是否完成路程
    int16_t accumulate_step;       // 累计步数
    int32_t max_step;              // 最大步数  - 配合累计步数以限幅
    int32_t min_step;              // 最小步数  - 配合累计步数以限幅
    uint32_t min_freq;             // 最小运行频率
    uint32_t max_freq;             // 最大运行频率

    // todo 函数接口层
    STEPMOTOR_INTERFACE limitStep;
    STEPMOTOR_INTERFACE noLimitStep;
    STEPMOTOR_STEP_INTERFACE stepMove;      // 指定步数和速度进行移动
    STEPMOTOR_INTERFACE stop;               // 立即停止
    STEPMOTOR_RANGE_INTERFACE setRange;     // 设置相关范围的接口

}STEPMOTOR;

extern STEPMOTOR stepm[STEP_SUM];

void stepmotor_attach(
        STEPID id,
        TIM_HandleTypeDef* htim,
        uint32_t Channel,           // 输出通道
        GPIO_TypeDef* enType,        //使能引脚类别
        uint16_t  enPin,             //使能引脚pin
        GPIO_TypeDef* dirType,      //方向引脚类别
        uint16_t  dirPin           //方向引脚pin
);
void stepmotor_init(void);
// 定时器更新中断处理（需在stm32f4xx_it.c中调用）
void Stepper_UpdateHandler(STEPID id);

#endif
