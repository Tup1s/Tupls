#include "Usart.h"
#include "stdio.h"
#define RX_BUFFER_SIZE 128
extern uint8_t rx_buffer[RX_BUFFER_SIZE];

uint8_t rx_buffer_uart2[1];
uint8_t rx_buffer_uart3[1];

extern uint8_t received_1;
extern uint8_t received_2;
extern uint8_t received_3;
extern uint8_t received_4;
extern uint8_t received_5;
extern uint8_t received_6;
extern uint8_t received_7;
extern uint8_t received_8;	

void USART_Init()
{
	__HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE);
	HAL_UART_Receive_DMA(&huart1,rx_buffer,RX_BUFFER_SIZE);

	HAL_UART_Receive_DMA(&huart2, rx_buffer_uart2,1);
    HAL_UART_Receive_IT(&huart3, rx_buffer_uart3,1);
}

int fputc(int ch, FILE *f)
{ 	
	uint8_t usart_tmp=(uint8_t)(ch);
	HAL_UART_Transmit(&huart1,&usart_tmp,1,1000);
//	HAL_UART_Transmit_DMA(&huart1,&usart_tmp,sizeof(usart_tmp));	
  return ch;
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)//串口回调函数
{
	if (huart->Instance == USART2) 
	{	 
	static uint8_t state = 0;
    static uint8_t data[8]={0};
	static uint8_t data_counter=0 ;
	uint8_t byte = rx_buffer_uart2[0];	
	switch (state) 
		{
        case 0: 
            if (byte == 0x2C) state++;	
            break;
        case 1: 
            if (byte == 0x12) state++;
            else state = 0;	
            break;
        case 2: 
             data[data_counter++] = byte;
            if (data_counter == 8) state++;	
            break;
        case 3: 
			if (byte == 0x5B) 
             {
				received_1=data[0];
				received_2=data[1];
				received_3=data[2];
				received_4=data[3];
				received_5=data[4];
				received_6=data[5];
				received_7=data[6];
				received_8=data[7];
				// printf("received_1:%d,received_2:%d\n", received_1,received_2);
				// printf("received_3:%d,received_4:%d\n", received_3,received_4);
				// printf("received_5:%d,received_6:%d\n", received_5,received_6);
				// printf("received_7:%d,received_8:%d\n", received_7,received_8);
			 }	
				state=0;
				data_counter=0;
		    break;
		}
	HAL_UART_Receive_DMA(&huart2, rx_buffer_uart2,1);
	}
}

// void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)//串口回调函数
// {
// 	if (huart->Instance == USART2) 
// 	{	
// 	static uint8_t state = 0;
//  static uint8_t data[8];
// 	static uint8_t data_counter=0 ;
// 	uint8_t byte = rx_buffer_uart2[0];	
// 	switch (state) 
// 		{
//         case 0: 
//              data[data_counter++] = byte;
//             if (data_counter == 8) state++;
//             break;
//         case 1: 
// 				received_1=data[0];
// 				received_2=data[1];
// 				received_3=data[2];
// 				received_4=data[3];
//              received_5=data[4];
// 				received_6=data[5];
// 				received_7=data[6];
// 				received_8=data[7];
// 				printf("received_1:%d,received_2:%d\r\n", received_1,received_2);
// 				printf("received_3:%d,received_4:%d\r\n", received_3,received_4);
//              printf("received_5:%d,received_6:%d\r\n", received_5,received_6);
// 				printf("received_7:%d,received_8:%d\r\n", received_7,received_8);
// 			state=0;
// 			data_counter=0;
// 		    break;
// 		}
// 	HAL_UART_Receive_DMA(&huart2, rx_buffer_uart2,1);
// 	}
// }
//usart2发送数据函数
// ---------------------------------------------------
// | 帧头(0xAA) | 帧头(0x55) | 数据长度 | 数据 | 校验和 |
// ---------------------------------------------------
void USART2_Send_Data(uint8_t *data, uint8_t length)
{
	uint8_t tx_buffer[3 + length]; 
	tx_buffer[0] = 0xAA; 
	tx_buffer[1] = 0x55; 
	tx_buffer[2] = length;

	for (int i = 0; i < length; i++)
	{
		tx_buffer[i + 3] = data[i]; 
	}
	uint8_t checksum = 0;
	for (int i = 0; i < length; i++)
	{
		checksum ^= data[i]; 
	}
	tx_buffer[3 + length] = checksum; 

	HAL_UART_Transmit_DMA(&huart2, tx_buffer, sizeof(tx_buffer)); 
}