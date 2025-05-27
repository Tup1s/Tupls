#ifndef __UART_H
#define __UART_H

#include "usart.h"
#include "stdio.h"



void USART_Init();
int fputc(int ch, FILE *f);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void USART2_Send_Data(uint8_t *data, uint8_t length);


#endif