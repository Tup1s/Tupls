#include "control.h"
//CH1为左轮CH2为右轮
void TIM_Init()
{
  HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_1|TIM_CHANNEL_2 );//Tim3编码器模式开启
  HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_1|TIM_CHANNEL_2 );//Tim4编码器模式开启
  
  HAL_TIM_Base_Start_IT(&htim2); // Tim2中断开启	
}
void TIM1_PWM_CH1_SetDuty(float duty) //设置电机占空比函数
	{
	if(duty < 0.0f) duty = 0.0f;
    if(duty > 100.0f) duty = 100.0f;
    uint32_t arr = TIM1->ARR;
    uint32_t ccr = (uint32_t)((duty / 100.0f) * (arr + 1));
    TIM1->CCR1 = ccr;
  }
void TIM1_PWM_CH2_SetDuty(float duty) 
	{
    if(duty < 0.0f) duty = 0.0f;
    if(duty > 100.0f) duty = 100.0f;
    uint32_t arr = TIM1->ARR;
    uint32_t ccr = (uint32_t)((duty / 100.0f) * (arr + 1));
    TIM1->CCR2 = ccr;
  }

	void Set_Motor_TIM1_CH1_Direction(int8_t Dir) 
{
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
  if (Dir==1) 
		{
			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
		} 
	else if(Dir==-1) 
		{
			HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    }
	else if(Dir==0)
	{
			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
			HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
  }
}
void Set_Motor_TIM1_CH2_Direction(int8_t Dir) 
{
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
  if (Dir==1) 
		{
			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
		} 
	else if(Dir==-1) 
		{
			HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    }
	else if(Dir==0)
	{
			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
			HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
  }
}

void Move_Forward(float duty) //前进
{
		Set_Motor_TIM1_CH1_Direction(1);    
		TIM1_PWM_CH1_SetDuty(duty);    
		
		Set_Motor_TIM1_CH2_Direction(-1);
		TIM1_PWM_CH2_SetDuty(duty);

}
	
void Move_Backward(float duty) //后退
{
		Set_Motor_TIM1_CH1_Direction(-1);  
		TIM1_PWM_CH1_SetDuty(duty);
	
		Set_Motor_TIM1_CH2_Direction(1);
		TIM1_PWM_CH2_SetDuty(duty);
		
}

void Turn_Left(float duty, float ratio) //左转（差速转向）通过ratio调节内外轮速差
{
  float left_duty = duty * (1 - ratio);
  float right_duty = duty * (1 + ratio);
  
  Set_Motor_TIM1_CH1_Direction(1);    
  TIM1_PWM_CH1_SetDuty(left_duty);

  Set_Motor_TIM1_CH2_Direction(-1);     
  TIM1_PWM_CH2_SetDuty(right_duty);

}

void Turn_Right(float duty, float ratio) //右转（差速转向）通过ratio调节内外轮速差
{
  float left_duty = duty * (1 + ratio);
  float right_duty = duty * (1 - ratio);
  
  Set_Motor_TIM1_CH1_Direction(1);    
  TIM1_PWM_CH1_SetDuty(left_duty);

  Set_Motor_TIM1_CH2_Direction(-1);     
  TIM1_PWM_CH2_SetDuty(right_duty);
}

void Stop() //停车
{
  TIM1_PWM_CH1_SetDuty(0);    
  TIM1_PWM_CH2_SetDuty(0);      
}

void Spin_CounterClockwise(float duty) //原地掉头（逆时针）
{
  Set_Motor_TIM1_CH1_Direction(-1);    
  TIM1_PWM_CH1_SetDuty(duty);

  Set_Motor_TIM1_CH2_Direction(-1);     
  TIM1_PWM_CH2_SetDuty(duty);

}

void Spin_Clockwise(float duty) 			//原地掉头（顺时针）
{
  Set_Motor_TIM1_CH1_Direction(1);    
  TIM1_PWM_CH1_SetDuty(duty);

  Set_Motor_TIM1_CH2_Direction(1);     
  TIM1_PWM_CH2_SetDuty(duty);

}
void example()
{
  //  Move_Forward(50);
  //  HAL_Delay(2000);
  //  Move_Backward(50 );
  //  HAL_Delay(2000);
  //  Stop();
  //  HAL_Delay(2000);
  //  Turn_Left(50, 0.2);
  //  HAL_Delay(2000);
  //  Turn_Right(50, 0.2);
  //  HAL_Delay(2000);
  //  Spin_CounterClockwise(50);
  //  HAL_Delay(2000);
  //  Spin_Clockwise(50);
  //  HAL_Delay(2000);
  //  Stop();
}
void TIM1_PWM_CH1_SetPWM(float pwm ) //设置电机pwm//左轮
	{
    TIM1->CCR1 = pwm ;
  }
void TIM1_PWM_CH2_SetPWM(float pwm ) //设置电机pwm//右轮
	{
    TIM1->CCR2 = pwm ;
  }