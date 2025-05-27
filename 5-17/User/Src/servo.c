#include "stm32h7xx_hal.h" 
#include "servo.h" 
#include "usart.h"

void SendServo_X_PWM(uint16_t pwm)
{
    char cmd[10];
    sprintf(cmd, "#00P%04d!", pwm);
//  HAL_UART_Transmit(&huart4, (uint8_t*)cmd, 9, HAL_MAX_DELAY);
	HAL_UART_Transmit_DMA(&huart4, (uint8_t*)cmd, 9);
}
void SendServo_Y_PWM(uint16_t pwm)
{
    char cmd[10];
    sprintf(cmd,"#01P%04d!", pwm);
//    HAL_UART_Transmit(&huart4, (uint8_t*)cmd, 9, HAL_MAX_DELAY);
    HAL_UART_Transmit_DMA(&huart4, (uint8_t*)cmd, 9);
}
