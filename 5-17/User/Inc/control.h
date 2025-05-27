#ifndef __CONTROL_H
#define __CONTROL_H

#include "tim.h"

void TIM_Init();
void TIM1_PWM_CH1_SetDuty(float duty);	
void TIM1_PWM_CH2_SetDuty(float duty);
void Set_Motor_TIM1_CH1_Direction(int8_t Dir);
void Set_Motor_TIM1_CH2_Direction(int8_t Dir);
void Move_Forward(float duty);
void Move_Backward(float duty);
void Turn_Left(float duty, float ratio);
void Turn_Right(float duty, float ratio);
void Stop();
void Spin_CounterClockwise(float duty);
void Spin_Clockwise(float duty);
void example();
void TIM1_PWM_CH1_SetPWM(float pwm );
void TIM1_PWM_CH2_SetPWM(float pwm );


#endif