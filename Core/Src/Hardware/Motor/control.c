//
// Created by jianai on 2024/12/28.
//
#include "control.h"
#include "stdio.h"

MOTOR_PID motor_speed_pid[M_SUM] = {
        {
                320.0f, 1.6f, 100.0f, 0.0f,
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

    if(m->err < 3 && m->err > -3){return m->output;}                           // 死区 : 因为是位置式 所以返回原值

    // todo 计算
    float output = 0;

    output =  pid->Kp * (m->err)
            + pid->Kd * (m->err - m->pre_err)
            + pid->Kf * (float)(m->tar_speed - m->pre_tar_speed);

    // todo 积分分离PID --> 当误差过大不启用积分
    if(m->err <= 15){
        output += pid->Ki * (m->integral);
        m->integral += m->err;                                  // 积分累加
        LIMIT(m->integral, pid->iMax, pid->iMin);   // 积分限幅
    }

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

        float rate = 0.65f;
        m->output = output * (1 - rate) + m->output * rate;
//        m->output = output;
        m->setSpeed(id, (int16_t)m->output);
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
