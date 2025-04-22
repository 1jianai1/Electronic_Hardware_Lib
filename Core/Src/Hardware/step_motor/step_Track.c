//
// Created by jianai on 2025/3/23.
//
#include "step_Track.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
// 参数初始化, 可以统一放在某个文件中单独调试,
//他是作为PID计算函数的参数使用的, 这里仅是初始化
Step_State stepState[STEP_SUM]={
        {0, 0,0,0,{0,0,0}, 0, 0, 0, 0, 0, 0, {-800, 130}},
        {0, 0, 0,0,{0,0,0},0, 0, 0, 0, 0, 0, {-100, 550}}
};

Step_PID stepLocatePid[STEP_SUM] = {
        {0.5f, 0.0f, 0.0f, 5.0f},
        {0.5f, 0.0f, 0.0f, 5.0f}
};



float Step_PID_Locate(const float* err, float* integral, float* pre_err, Step_PID* pid){

    float output =(                                                  // 增量式计算
            pid->kp * (*err) +
            pid->ki * (*integral) +
            pid->kd * ((*err) - (*pre_err)) /// 0.01f
    );
    *pre_err = *err;    // 误差缓存

    *integral += *err;  //积分累加
    // 积分限幅
    if(*integral>200)*integral=200;
    else if(*integral<-200)*integral=-200;

//    static float pre_output = 0;
//    float rate = 0.7f;
//    float temp = rate*pre_output + (1-rate)*output;
//    pre_output = output;
    return output;
}

//{500.0f, 0.00f, 15.0f, 9.0f},
//{500.0f, 0.00f, 15.0f, 9.0f}
//{2.7f, 3.04f, 25.0f, 0.00f},
//{2.65f, 3.5f, 25.0f, 0.00f}
Step_PID stepSpeedPid[STEP_SUM] = {
{2.7f, 3.04f, 25.0f, 0.00f},
{2.65f, 3.5f, 25.0f, 0.00f}
};
//// α-β滤波器参数（需根据目标运动特性调整）
//#define ALPHA 0.6f
//#define BETA 0.2f
//#define dt 0.01f
//#define ONE_STEP 2
//void Step_Speed_Track_Calculate(Step_State* state){
//    for(uint8_t i=0; i < STEP_SUM; i++){
//        // todo 预测项
//        state[i].prev_target[2] = state[i].prev_target[1];
//        state[i].prev_target[1] = state[i].prev_target[0];
//        state[i].prev_target[0] = state[i].target; // 最新目标
//        // α-β滤波器估计目标速度和加速度
//        if(state[i].prev_target[2] != 0) { // 需至少3个历史点
//            float vel = (state[i].prev_target[0] - state[i].prev_target[1]) / dt;
//            float vel_prev = (state[i].prev_target[1] - state[i].prev_target[2]) / dt;
//            state[i].target_vel += ALPHA * (vel - state[i].target_vel);
//            state[i].target_acc = BETA * (vel - vel_prev) / dt;
//        }
//        // 计算前馈
//        float feedforward = state[i].target_vel * stepSpeedPid[i].kf;
//
//        // todo 反馈项
//        float error = state[i].target-state[i].actual;                     // 得到误差
//        if(fabsf(error) < 5){
//            state[i].freq = 0;
//            continue;
//        }
//
//        float delta =
//                    stepSpeedPid[i].kp * (error - state[i].pre_err)
//                +   stepSpeedPid[i].ki * error * dt
//                +   stepSpeedPid[i].kd * (error - 2 * state[i].pre_err + state[i].pre_pre_err) * dt
//                ;
//
//        state[i].pre_pre_err = state[i].pre_err;
//        state[i].pre_err = error;
//
//        // todo 整合
//        state[i].output = feedforward + delta;
//
//        //todo 限幅 和滤波
////        state[i].output = Step_PID_Locate(
////                &error,
////                &state[i].integral,
////                &state[i].pre_err,
////                &stepSpeedPid[i]
////                );
////        state[i].output += (stepSpeedPid[i].kf * (state[i].target - state[i].pre_target));
////        state[i].target = state[i].pre_target;
//
//        if(i == 0) KalmanFilter_x(&kfpVar_x, state[Yaw].output);
//        else if(i == 1) KalmanFilter_y(&kfpVar_y, state[Pitch].output);
//
//        state[i].freq = (int)state[i].output;
//    }
//    step_move(
//            Yaw,
//              (state[Yaw].target-state[Yaw].actual)<0?ONE_STEP:-ONE_STEP,
//              abs(state[Yaw].freq)
//              );
//    step_move(
//            Pitch,
//              (state[Pitch].target-state[Pitch].actual)>0?ONE_STEP:-ONE_STEP,
//              abs(state[Pitch].freq)
//              );
//}
//
//void Step_LocateAdd_Track_Calculate(Step_State* state){
//    for(uint8_t i = 0; i < STEP_SUM; i++){
//        float error = (state[i].target - state[i].actual) * TRANS;        //
//        if(error < 5*TRANS && error > -5*TRANS)continue;
//
//        state[i].output =
//                stepLocatePid[i].kp * (error ) +
//                stepLocatePid[i].ki * (error) +
//                stepLocatePid[i].kd * (error - 2 * state[i].pre_err + state[i].pre_pre_err)
//                ;
//
//        state[i].pre_pre_err = state[i].pre_err;
//        state[i].pre_err = error;
//
//        if(state[i].output > MAX_STEP){
//            state[i].output = MAX_STEP;
//        }else if(state[i].output < -MAX_STEP){
//            state[i].output = -MAX_STEP;
//        }
//        if(i == 0) KalmanFilter_x(&kfpVar_x, state[Yaw].output);
//        else if(i == 1) KalmanFilter_y(&kfpVar_y, state[Pitch].output);
//
////        if(i == Pitch)
////            state[i].output = -state[i].output;
//        if(i == Yaw)
//            step_move(i, (int16_t)state[i].output , 200);
//    }
//}
//
//void Step_Track_SetTarget(Step_State* state , Point* target_point){
//    state[Yaw].target = (target_point->x); // 像素转角度
//    state[Pitch].target = (target_point->y);
//
//}
//
//void Step_Track_SetActual(Step_State* state , Point* actual_point){
//    state[Yaw].actual = (actual_point->x); // 像素转角度
//    state[Pitch].actual = (actual_point->y);
//}