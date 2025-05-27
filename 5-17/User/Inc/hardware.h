#ifndef __HARDWARE_H
#define __HARDWARE_H
#include "gpio.h"

#define red 0x01
#define green 0x02
#define blue 0x03

void LED_ON();
void LED_OFF();
void LED_TOGGLE();
uint8_t	KEY_Scan(void);
void BUZZER_ON();
void BUZZER_OFF();
void LASER_ON();
void LASER_OFF();
void STATE(uint8_t state);
void test();
#endif