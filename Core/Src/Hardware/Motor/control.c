//
// Created by jianai on 2024/12/28.
//
#include "control.h"
#include "stdio.h"
#include "filter.h"

MOTOR_PID motor_speed_pid[M_SUM] = {
        {
                750.0f, 0.1f, 70.0f, 0.0f,
                1000.0f, -1000.0f,
                8000.0f, -8000.0f
        },
};//速度环pid1

static float LIMIT(float ac, float ma, float mi){
    if(ac > ma) return ma;
    else if(ac < mi) return mi;
    return ac;
}

// 公式实现 位置式
static float pid_calculate(MotorID id){
    MOTOR* m = &motor[id];
    MOTOR_PID* pid = &motor_speed_pid[id];

    m->err = (float)(m->tar_speed - m->en_actual);      // 计算误差

    if(m->err < 4){return 0;}                           // 死区

    // 计算
    float output =
              pid->Kp * (m->err)
            + pid->Ki * (m->integral)
            + pid->Kd * (m->err - m->pre_err)
            + pid->Kf * (float)(m->tar_speed - m->pre_tar_speed);

    LIMIT(m->integral, pid->iMax, pid->iMin);   // 积分限幅

    m->integral += m->err;                                  // 积分累加
    m->pre_err = m->err;                                    // 误差赋值
    m->pre_tar_speed = m->tar_speed;                        // 前馈目标

    return output;
}

static void Motor_PID_Calculation(MotorID id){
        MOTOR* m = &motor[id];

        float output;
        // 你还可以添加其他的环进行串级 或者 并级控制
        // 内环 - 速度环实现
        output = pid_calculate(id);

        // 输出限幅
        LIMIT(output, motor_speed_pid[id].oMax, motor_speed_pid[id].oMin);

        m->output = output;
        m->setSpeed(id, (int)m->output);
}

static void set_target_speed(MotorID id, int16_t speed){
    motor[id].tar_speed = speed;
}


void Motor_PID_init(void){
    // 对每个对象的接口进行指定
    for(uint8_t i = 0; i < M_SUM; i++){
        motor[i].setTarSpeed = set_target_speed;        // PID 速度环赋目标值
        motor[i].pidSpeedloop = Motor_PID_Calculation;  // PID 计算循环调用
    }
}
