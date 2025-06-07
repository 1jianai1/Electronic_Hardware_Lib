//
// Created by jianai on 2025/1/2.
//

#ifndef REMOTE_VET_PROJ_FILTER_H
#define REMOTE_VET_PROJ_FILTER_H


typedef struct
{
    float P; //估算协方差
    float G; //卡尔曼增益
    float Q; //过程噪声协方差,Q增大，动态响应变快，收敛稳定性变坏
    float R; //测量噪声协方差,R增大，动态响应变慢，收敛稳定性变好
    float Output; //卡尔曼滤波器输出
}KFPTypeS; //Kalman Filter parameter type Struct
extern KFPTypeS kfpVar;
float KalmanFilter(KFPTypeS* kfp, float input);
#endif //REMOTE_VET_PROJ_FILTER_H
