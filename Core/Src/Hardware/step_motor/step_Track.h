//
// Created by jianai on 2025/3/23.
//

#ifndef GREEN_PRJ_STEP_TRACK_H
#define GREEN_PRJ_STEP_TRACK_H
#include "main.h"
#include "../Src/Hardware/step_motor/step_motor.h"

#define TRANS   1.86f
#define MAX_STEP  50

typedef struct{
    float kp; // Proportional gain
    float ki; // Integral gain
    float kd; // Derivative gain
    float kf;
} Step_PID;

typedef struct{

    float actual;     // 实际位置 -- 视觉模块得到
    float target;     // 目标位置 -- 作为PID计算参数,

    float target_vel;   // 预测的目标速度（前馈用）
    float target_acc;   // 预测的目标加速度（前馈用）
    float prev_target[3]; // 历史目标位置（用于速度估计）

    float pre_target;
    float pre_err;    // 上一次误差
    float pre_pre_err;// 上上一次误差

    float integral;   // 积分项 -- 增量式不用
    float output;     // PID计算结果
    int freq;        // freq()
    int  range[2];   // 取值范围 -- 0 为极小值下标, 1 为极大值下标

} Step_State;

extern Step_State stepState[STEP_SUM];
extern Step_PID stepSpeedPid[STEP_SUM];
//void Step_Speed_Track_Calculate(Step_State* state);
//void Step_LocateAdd_Track_Calculate(Step_State* state);
//void Step_Track_SetTarget(Step_State* state , Point* target_point);
//void Step_Track_SetActual(Step_State* state , Point* actual_point);
#endif //GREEN_PRJ_STEP_TRACK_H
