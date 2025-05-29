#include "stm32h7xx_hal.h" 
#include "cascade_control.h"
#include <stdio.h>

extern int32_t position_TIM3 ; // 编码器计数位置
extern int32_t position_TIM4 ; 
extern float speed_TIM3 ;// 电机1线速度
extern float speed_TIM4 ;// 电机2线速度
 
// 用于初始化PID参数的函数
void PID_Init_control(PID *pid, float p, float i, float d, float maxI, float maxOut)
{
    pid->kp = p;  // 设置比例系数
    pid->ki = i;  // 设置积分系数
    pid->kd = d;  // 设置微分系数
    pid->maxIntegral = maxI;  // 设置积分限幅
    pid->maxOutput = maxOut;  // 设置输出限幅
    pid->error = 0;
    pid->lastError = 0;
    pid->integral = 0;
    pid->output = 0;
}
 
// 参数为(pid结构体, 目标值, 反馈值)，计算结果放在pid结构体的output成员中
void PID_Calc(PID *pid, float  reference, float  feedbackposition)
{
    // 更新数据 
    pid->lastError = pid->error;  // 将旧error存起来
    pid->error = reference - feedbackposition;  // 计算新error
 
    // 计算微分项
    float dout = (pid->error - pid->lastError) * pid->kd;
 
    // 计算比例项
    float pout = pid->error * pid->kp;
 
    // 计算积分项
    pid->integral += pid->error * pid->ki;
 
    // 积分限幅
    if (pid->integral > pid->maxIntegral) 
        pid->integral = pid->maxIntegral;
    else if (pid->integral < -pid->maxIntegral) 
        pid->integral = -pid->maxIntegral;
 
    // 计算输出
    pid->output = pout + dout + pid->integral;
 
    // 输出限幅
    if (pid->output > pid->maxOutput) 
        pid->output = pid->maxOutput;
    else if (pid->output < -pid->maxOutput) 
        pid->output = -pid->maxOutput;
}

// 参数(PID结构体, 外环目标值, 外环反馈值, 内环反馈值)
void PID_CascadeCalc(CascadePID *pid, float outerRef, float outerFdb, float innerFdb)
{
    PID_Calc(&pid->outer, outerRef, outerFdb); // 计算外环
    PID_Calc(&pid->inner, pid->outer.output, innerFdb); // 计算内环
    pid->output = pid->inner.output; // 内环输出就是串级PID的输出
}
 
// 模拟设定执行器输出大小的函数
void setActuatorOutput(TIM_HandleTypeDef *htim,float output)
{
    if (htim->Instance == TIM3){
        TIM1_PWM_CH2_SetPWM(output);
       // printf("TIM3 Output: %f\n", output);//右轮PWM
    } 
    if (htim->Instance == TIM4){
        TIM1_PWM_CH1_SetPWM(output);
       // printf("TIM4 Output: %f\n", output);//左轮PWM
    } 
}

// 模拟获取反馈值的函数
float getFeedbackPosition(TIM_HandleTypeDef *htim)
{
    float getposition;
     if (htim->Instance == TIM3){
        getposition = position_TIM3;
       // printf("TIM3 Output: %f\n", getposition);//右轮
    } 
    if (htim->Instance == TIM4){
         getposition = position_TIM4;
       //printf("TIM4 Output: %f\n", getposition);//左轮
    } 
    return getposition;
}
 float getFeedbackSpeed(TIM_HandleTypeDef *htim)
{
    float getspeed;
     if (htim->Instance == TIM3){
        getspeed = speed_TIM3;
       // printf("TIM3 Output: %f\n", getspeed);//右轮
    } 
    if (htim->Instance == TIM4){
         getspeed = speed_TIM4;
        //printf("TIM4 Output: %f\n", getspeed);//左轮
    } 
    return getspeed;
}
 
// 模拟获取目标值的函数
float getTargetPosition(TIM_HandleTypeDef *htim,int32_t position)
{
    float targetPosition;
       if (htim->Instance == TIM3){
        targetPosition = position;
       // printf("TIM3 Output: %f\n", targetPosition);//右轮
    } 
    if (htim->Instance == TIM4){
         targetPosition = position;
       // printf("TIM4 Output: %f\n", targetPosition);//左轮
    } 
    return targetPosition;
}

    
 

 