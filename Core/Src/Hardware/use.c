//
// Created by jianai on 2025/6/8.
//

#include "use.h"
#include <stdio.h>
/*
 * todo 各种模块的使用示例
 * */

// todo QMC5883
#include "Magnetic/QMC5883/qmc5883.h"
void use_qmc5883(void){
    qmc5883.init();
    for(;;){

        printf("qmc:%d, %d\r\n", qmc5883.getYaw(), qmc5883.getTemp());
        HAL_Delay(10);
    }
}

// todo dmp_mpu6050
#include "IMU/MPU/MPU6050/mpu6050.h"
#include "IMU/MPU/DMP/inv_mpu.h"
void use_dmp_mpu6050(void){
    MPU6050_Init();
    mpu_dmp_init();
    float pitch, yaw, roll;
    for(;;){
         mpu_dmp_get_data(&pitch, &roll, &yaw);
         printf("angle: %d, %d, %d\r\n", (int)pitch, (int)roll, (int)yaw);
    }
}

// todo motor 直流编码电机
#include "Motor/control.h"
void use_motor(void){
    motor_attach(
            M1,
            &htim3,
            TIM_CHANNEL_1,
            &htim4,
            GPIOB,
            GPIO_PIN_4,
            GPIOB,
            GPIO_PIN_5
    );
    motor_init();
    Motor_PID_init();
    motor[M1].setTarSpeed(M1, 25);

    uint16_t time = 0, idx = 0;
    int16_t speed_arr[10] = {-1, 5 ,-10, 15, -20, 15, -10, 5, 0, -10};
    motor[M1].setTarSpeed(M1, speed_arr[0]);
    for(;;){
        if(time > 150){
            time = 0;
            idx++;
            if(idx > 9)idx = 0;
            motor[M1].setTarSpeed(M1, speed_arr[idx]);
        }
        time++;
        printf("speed: %d, %d\r\n", motor[M1].en_actual, motor[M1].tar_speed);
        HAL_Delay(10);
    }
}
// 放在定时中断循环中，定时进行pid控制
void motor_tim_loop(void){
    motor[M1].getSpeed(M1);
    motor[M1].pidSpeedloop(M1);
}

// todo 使用TB6600驱动的步进电机
#include "step_motor/step_Track.h"
void use_stepmotor(void){
    stepmotor_attach(
            SM1,
            &htim5,
            TIM_CHANNEL_3,
            GPIOB,
            GPIO_PIN_1,
            GPIOA,
            GPIO_PIN_5
            );
    stepmotor_init();
    stepm[SM1].stepMove(SM1, -1000, 1000);
    HAL_Delay(500);
    stepm[SM1].stop(SM1);
    HAL_Delay(1000);
    stepm[SM1].stepMove(SM1, 1000, 1000);
}

