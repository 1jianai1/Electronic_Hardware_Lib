//
// Created by jianai on 2024/11/12.
//
#include "motor.h"
#include <stdio.h>


//电机硬件宏参数赋值
MOTOR_HARDWARE motorhardware[M_SUM];
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

// todo 引脚1拉高
static void pin1_set(MotorID id){
    MOTOR_HARDWARE* hw = &motorhardware[id];
    hw->GPIOx_1->BSRR = hw->M_IN1;
}
// todo 引脚1拉低
static void pin1_reset(MotorID id){
    MOTOR_HARDWARE* hw = &motorhardware[id];
    hw->GPIOx_1->BSRR = (uint32_t)hw->M_IN1<<16U;
}
// todo 引脚2拉高
static void pin2_set(MotorID id){
    MOTOR_HARDWARE* hw = &motorhardware[id];
    hw->GPIOx_2->BSRR = hw->M_IN2;
}
// todo 引脚2拉低
static void pin2_reset(MotorID id){
    MOTOR_HARDWARE* hw = &motorhardware[id];
    hw->GPIOx_2->BSRR = (uint32_t)hw->M_IN1<<16U;
}
// todo 指定频率, 且保证占空比不变
static void set_freq(MotorID id, uint32_t freq){
    MOTOR_HARDWARE* hw = &motorhardware[id];
    // 取当前占空比 = 比较值/自动重装载值
    if(freq == 0) return;
    float duty = __HAL_TIM_GET_COMPARE(hw->htim, hw->Channel)/ (__HAL_TIM_GET_AUTORELOAD(hw->htim)*1.0);
    __HAL_TIM_SET_AUTORELOAD(hw->htim, ((MOTOR_Hclk)/freq) - 1 );
    __HAL_TIM_SET_COMPARE(hw->htim, hw->Channel, (uint32_t)(duty * __HAL_TIM_GET_AUTORELOAD(hw->htim)));
}
// todo 指定比较值
static void pwm_setcompare(MotorID id, uint32_t compare){
    MOTOR_HARDWARE* hw = &motorhardware[id];
    if(compare >= __HAL_TIM_GET_AUTORELOAD(hw->htim))
        compare = __HAL_TIM_GET_AUTORELOAD(hw->htim) - 1;
    __HAL_TIM_SET_COMPARE(hw->htim, hw->Channel, compare);
}
// todo PWM启动
static void pwm_start(MotorID id){
    MOTOR_HARDWARE* hw = &motorhardware[id];
    HAL_TIM_PWM_Start(hw->htim, hw->Channel);
}
// todo PWM停止
static void pwm_stop(MotorID id){
    MOTOR_HARDWARE* hw = &motorhardware[id];
    HAL_TIM_PWM_Stop(hw->htim, hw->Channel);
}
// todo 编码器启动
static void encoder_start(MotorID id){
    MOTOR_HARDWARE* hw = &motorhardware[id];
    HAL_TIM_Encoder_Start(hw->entim,TIM_CHANNEL_ALL);

}
// todo 编码器停止
static void encoder_stop(MotorID id){
    MOTOR_HARDWARE* hw = &motorhardware[id];
    HAL_TIM_Encoder_Stop(hw->entim,TIM_CHANNEL_ALL);
}
// todo 编码器清零
static void encoder_reset(MotorID id){
    MOTOR_HARDWARE* hw = &motorhardware[id];
    __HAL_TIM_SET_COUNTER(hw->entim, 0);
}
// todo 获取编码器值
static void encoder_get(MotorID id){
    MOTOR_HARDWARE* hw = &motorhardware[id];
    motor[id].en_actual = (short)(__HAL_TIM_GET_COUNTER(hw->entim));
}



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
            motor[id].hw->setPwm(id, speed);
        }
        else{
            M_N(id);
            motor[id].hw->setPwm(id, -speed);
        }
}

static void get_speed(MotorID id){
//    motor[id].en_actual = (short)__HAL_TIM_GET_COUNTER(motor[id].hw->entim);
    motor[id].hw->enGet(id);
    //编码器寄存器置0
    motor[id].hw->enReset(id);
}

// 结构体引脚定向
void motor_attach(
        MotorID id,
        TIM_HandleTypeDef* htim,  //定时器句柄
        uint32_t tim_channel,    //CH通道
        TIM_HandleTypeDef* entim,
        GPIO_TypeDef* GPIOx_1,
        uint16_t M_IN1,
        GPIO_TypeDef* GPIOx_2,
        uint16_t M_IN2
){

    MOTOR_HARDWARE *hw = &motorhardware[id];
    // todo 引脚重定向
    hw->htim = htim;                  // 定时器定向
    hw->Channel = tim_channel;    // CH通道定向
    hw->entim = entim;                // 编码器
    hw->GPIOx_1 = GPIOx_1;
    hw->GPIOx_2 = GPIOx_2;            // 方向通道定向
    hw->M_IN1 = M_IN1;
    hw->M_IN2 = M_IN2;

    // todo 硬件操作层函数重定向 - 你当然也可以自定义函数指针添加进来 前提是得满足声明要求
    hw->pin1Set = pin1_set;
    hw->pin1Reset = pin1_reset;
    hw->pin2Set = pin2_set;
    hw->pin2Reset = pin2_reset;
    hw->setFreq = set_freq;
    hw->setPwm = pwm_setcompare;
    hw->pwmStart = pwm_start;
    hw->pwmStop = pwm_stop;
    hw->enStart = encoder_start;
    hw->enStop = encoder_stop;
    hw->enReset = encoder_reset;
    hw->enGet = encoder_get;

    // todo 控制函数指针重定向
    MOTOR* id_motor = &motor[id];
    id_motor->hw = hw;
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
        motor[i].hw->psc = motor[i].hw->htim->Instance->PSC;
        motor[i].hw->arr = motor[i].hw->htim->Instance->ARR;

        //启动 PWM + 编码器定时器
        motor[i].hw->pwmStart(i);
        motor[i].hw->enStart(i);
//        HAL_TIM_PWM_Start(motor[i].hw->htim,motor[i].hw->Channel);
//        HAL_TIM_Encoder_Start(motor[i].hw->entim,TIM_CHANNEL_ALL);
    }
}
