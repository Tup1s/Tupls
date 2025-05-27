#ifndef __CASCADE_H
#define __CASCADE_H

#include "tim.h"
#include "control.h"

typedef struct
{
    float kp, ki, kd;        // 三个系数：比例、积分和微分
    float error, lastError;  // 当前误差、上次误差
    float integral, maxIntegral;  // 积分、积分限幅
    float output, maxOutput; // 输出、输出限幅
} PID;
typedef struct
{
    PID inner; // 内环
    PID outer; // 外环
    float output; // 串级输出，等于inner.output
} CascadePID;

extern CascadePID pid_TIM3 ; // 创建串级PID结构体变量
extern CascadePID pid_TIM4 ; 

void PID_Init_control(PID *pid, float p, float i, float d, float maxI, float maxOut);
void PID_Calc(PID *pid, float  reference, float  feedbackposition);
void PID_CascadeCalc(CascadePID *pid, float outerRef, float outerFdb, float innerFdb);
void setActuatorOutput(TIM_HandleTypeDef *htim,float output);
float getFeedbackPosition(TIM_HandleTypeDef *htim);
float getFeedbackSpeed(TIM_HandleTypeDef *htim);
float getTargetPosition(TIM_HandleTypeDef *htim,int32_t position);







#endif