#include "stm32h7xx_hal.h" 
#include "PID.h"

void PID_Init(PID_Controller *pid, float kp, float ki, float kd)  
{  
    pid->Setpoint = 0.0f;  
    pid->Input = 0.0f;  
    pid->Output = 0.0f;  
    pid->Kp = kp;  
    pid->Ki = ki;  
    pid->Kd = kd;  
    pid->LastError = 0.0f;  
    pid->Integral = 0.0f;  
}  
float PID_Compute(PID_Controller *pid, float input)  
{  
    float error = pid->Setpoint - input;  
    pid->Integral += error;  
    float derivative = error - pid->LastError; 
    pid->Output = pid->Kp * error + pid->Ki * pid->Integral + pid->Kd * derivative;  
    pid->LastError = error;  
    return pid->Output;  
}  

void PID_SetInput(PID_Controller *pid, float input)  
{  
    pid->Input = input;  
}  
  
void PID_SetSetpoint(PID_Controller *pid, float setpoint)  
{  
    pid->Setpoint = setpoint;  
}
uint16_t angle_to_pulse(float angle) 
{
  angle = angle > 180 ? 180 : (angle < 0 ? 0 : angle);
  return (uint16_t)(500 + (angle / 180.0) * 2000); 
}  
