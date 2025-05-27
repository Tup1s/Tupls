#ifndef __SERVO_H
#define __SERVO_H

#include "usart.h"
#include "stdio.h"


void SendServo_X_PWM(uint16_t pwm);
void SendServo_Y_PWM(uint16_t pwm);


#endif