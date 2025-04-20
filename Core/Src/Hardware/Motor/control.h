//
// Created by jianai on 2024/12/28.
//

#ifndef REMOTE_VET_PROJ_CONTROL_H
#define REMOTE_VET_PROJ_CONTROL_H

#include "main.h"
#include "motor.h"

typedef struct
{
    float Kp, Ki, Kd, Kf;          	//定义比例、积分、微分系数、前馈
    float iMax, iMin;                // 微分限幅
    float oMax, oMin;               // 输出限幅
}MOTOR_PID;

extern MOTOR_PID motor_speed_pid[M_SUM];    //速度环pid参数

void Motor_PID_init(void);
#endif //REMOTE_VET_PROJ_CONTROL_H
