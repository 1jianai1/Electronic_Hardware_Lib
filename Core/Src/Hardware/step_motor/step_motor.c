//
// Created by jianai on 2025/4/21.
//
#include "step_motor.h"
#include <stdio.h>
#include <stdlib.h>

STEPMOTOR_HARDWARE stepmotorhws[STEP_SUM];
STEPMOTOR stepm[STEP_SUM] = {
        {
                &stepmotorhws[SM1],
                0,
                0,
                1,
                0,
                0,
                0,
                0,
                1000,
                -1000,
                0,
                10000
            }
};

static int32_t LIMIT(int32_t ac, int32_t ma, int32_t mi){
    if(ma < mi) return 0;
    return  ac > ma ? ma :
            ac < mi ? mi : ac;
}

static void en_set(STEPID id){// todo 使能置高
    STEPMOTOR_HARDWARE* hw = &stepmotorhws[id];
    hw->enType->BSRR = hw->enPin;
}
static void en_reset(STEPID id){// todo 使能置低
    STEPMOTOR_HARDWARE* hw = &stepmotorhws[id];
    hw->enType->BSRR = (uint32_t)hw->enPin << 16U;                 // 置位
}

static void dir_set(STEPID id){// todo 方向置高
    STEPMOTOR_HARDWARE* hw = &stepmotorhws[id];
    hw->dirType->BSRR = hw->dirPin;
}
static void dir_reset(STEPID id){// todo 方向置低
    STEPMOTOR_HARDWARE* hw = &stepmotorhws[id];
    hw->dirType->BSRR = (uint32_t)hw->dirPin << 16U;                 // 置位
}

static void tim_start(STEPID id){// todo 定时器启动
    STEPMOTOR_HARDWARE* hw = &stepmotorhws[id];
    hw->htim->Instance->CR1 |= TIM_CR1_CEN;
}
static void tim_stop(STEPID id){// todo 定时器停止并清0
    STEPMOTOR_HARDWARE* hw = &stepmotorhws[id];
    __HAL_TIM_SET_COUNTER(hw->htim, 0);
    __HAL_TIM_CLEAR_FLAG(hw->htim, TIM_FLAG_UPDATE);
}

static void pwm_start(STEPID id){// todo pwm启动
    STEPMOTOR_HARDWARE* hw = &stepmotorhws[id];
    HAL_TIM_PWM_Start(hw->htim, hw->Channel);
}
static void pwm_stop(STEPID id){// todo pwm停止 其实将比较值赋为0就行
    STEPMOTOR_HARDWARE* hw = &stepmotorhws[id];
    HAL_TIM_PWM_Stop(hw->htim, hw->Channel);
}

static void pwm_startIT(STEPID id){// todo pwm中断开启
    STEPMOTOR_HARDWARE* hw = &stepmotorhws[id];
    HAL_TIM_PWM_Start_IT(hw->htim, hw->Channel);
}
static void pwm_stopIT(STEPID id){// todo pwm中断停止
    STEPMOTOR_HARDWARE* hw = &stepmotorhws[id];
    HAL_TIM_PWM_Stop_IT(hw->htim, hw->Channel);
}
static void pwm_setcompare(STEPID id, int32_t freq){// todo 设置50%的占空比
    // todo 重新设置占空比(占空比永远为比较值的一半, 即50%占空比)
    STEPMOTOR_HARDWARE* hw = &stepmotorhws[id];
    if(freq > 0){
        hw->htim->Instance->ARR = Hclk/freq - 1;
        __HAL_TIM_SET_COMPARE(hw->htim, hw->Channel, hw->htim->Instance->ARR/2); // 占空比无所谓 一半即可
    }
}

static void limit_step(STEPID id){stepm[id].is_limit_step = 1;}
static void no_limit_step(STEPID id){stepm[id].is_finish = 0;}

// todo 以指定速度(频率), 移动指定步数
static void step_move(STEPID id, int32_t tar_steps, uint32_t freq){
    STEPMOTOR* m = &stepm[id];

    if(tar_steps == 0 || freq <=0){m->is_finish = 1;return;}
    m->is_finish = 0;   // 刷新完成标志位

    // 停止并复位定时器
    m->hw->pwmStopIT(id);
    m->hw->htimStop(id);

    //todo 方向引脚置位
    m->dir = (tar_steps >= 0) ? 1 : 0;
    if(m->dir) m->hw->dirSet(id);
    else m->hw->dirReset(id);

    // todo 设置目标步数 并对其进行步数限幅
    if(m->is_limit_step){
        int32_t pre_accumulate_step = m->accumulate_step;  // 存储上一次的步数
        m->accumulate_step = (int16_t) LIMIT(m->accumulate_step + tar_steps, m->max_step, m->min_step);
        m->tar_step = abs( m->accumulate_step - pre_accumulate_step);
    }else{
        m->tar_step = tar_steps;
    }

    m->cur_step = 0;    // 每刷新一次当前步数置为0

    // todo 限制频率范围  注意符号和取值范围
    freq = LIMIT((int32_t)freq, (int32_t)m->max_freq, (int32_t)m->min_freq);
    m->cur_freq = freq; // 获取当前频率

    // todo 重新设置占空比(占空比永远为比较值的一半, 即50%占空比)
    m->hw->pwmSetCompare(id, freq);
    // todo 重新启动定时器
    m->hw->htimStart(id);

    // todo 启用更新中断
    m->hw->pwmStartIT(id);
}

// todo 立即停止电机
static void step_stop(STEPID id) {
    STEPMOTOR * m = &stepm[id];
    //__HAL_TIM_DISABLE_IT(m->hw->htim, TIM_IT_UPDATE);
    m->tar_step = m->cur_step;
}

static void set_range(STEPID id, char mode, int32_t ma, int32_t mi){
    STEPMOTOR* m = &stepm[id];
    switch (mode) {
        case 'f':   // todo 设置频率范围
            m->max_freq = ma;
            m->min_freq = mi;
            break;
        case 's':
            m->max_step = ma;
            m->min_step = mi;
            break;
    }
}

// todo (重要)定时器更新中断处理（需在stm32f4xx_it.c(或者中断回调函数)中调用）
void Stepper_UpdateHandler(STEPID id) {
    STEPMOTOR* m = &stepm[id];
    if(m->cur_step < m->tar_step) {
        m->cur_step++;
    } else {                            // todo 如果检测到当前步数到了目标值就直接PWM_Stop不发波了
        m->is_finish = 1;
        m->hw->pwmStopIT(id);
    }
}

//输出比较的回调函数
//原理：当cnt加到和ccr一样时发出一个脉冲，通过不断改变ccr的值则可以在65535范围内产生多个脉冲 16位定时器cnt最大能达到65535，,32为可达到4294697295
//此种模式下相关步进电机的计算
//1.步进电机的频率：1/(PSC/HCLK(主频：84KM)*X(偏移量))


void stepmotor_attach(
        STEPID id,
        TIM_HandleTypeDef* htim,
        uint32_t Channel,           // 输出通道
        GPIO_TypeDef* enType,        //使能引脚类别
        uint16_t  enPin,             //使能引脚pin
        GPIO_TypeDef* dirType,      //方向引脚类别
        uint16_t  dirPin           //方向引脚pin
        ){
    STEPMOTOR_HARDWARE* hw = &stepmotorhws[id];
    // todo 硬件引脚重指定
    hw->htim = htim;            // 硬件接口初始化
    hw->Channel = Channel;
    hw->enType = enType;
    hw->enPin = enPin;
    hw->dirType = dirType;
    hw->dirPin = dirPin;

    // todo 硬件函数接口
    hw->enSet = en_set;
    hw->enReset = en_reset;
    hw->dirSet = dir_set;
    hw->dirReset = dir_reset;
    hw->htimStart = tim_start;
    hw->htimStop = tim_stop;
    hw->pwmStop = pwm_stop;
    hw->pwmStart = pwm_start;
    hw->pwmStartIT = pwm_startIT;
    hw->pwmStopIT = pwm_stopIT;
    hw->pwmSetCompare = pwm_setcompare;

    // todo 控制接口函数
    stepm[id].hw = hw;
    stepm[id].limitStep = limit_step;
    stepm[id].noLimitStep = no_limit_step;
    stepm[id].stepMove = step_move;    // 移动函数接口赋值
    stepm[id].stop = step_stop;        // 立即停止接口指定
    stepm[id].setRange = set_range;    // 范围设置指向

}

void stepmotor_init(void){
    for(uint8_t i = 0; i < STEP_SUM; i++){
        STEPMOTOR* m = &stepm[i];                      // 获取对象指针
        // todo 其他初始化

        // todo 启动
        m->hw->pwmStartIT(i);
        m->hw->enSet(i);
    }
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if(htim == stepm[SM1].hw->htim)    // 回调函数检测为某个步进电机对应的定时器
    {
        Stepper_UpdateHandler(SM1);
    }
}
