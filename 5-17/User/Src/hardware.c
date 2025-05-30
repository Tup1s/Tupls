#include "hardware.h"

void LED_ON()
{
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_RESET);
}
void LED_OFF()
{
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_SET);
}
void LED_TOGGLE()
{
	HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_2);	
}
uint8_t	KEY_Scan(void)
{
	if( HAL_GPIO_ReadPin ( GPIOE,GPIO_PIN_3) == 0 )	//��ⰴ���Ƿ񱻰���
	{	
		HAL_Delay(10);	//��ʱ����
		if( HAL_GPIO_ReadPin ( GPIOE,GPIO_PIN_3) == 0)	//�ٴμ���Ƿ�Ϊ�͵�ƽ
		{
			while(  HAL_GPIO_ReadPin ( GPIOE,GPIO_PIN_3) == 0);	//�ȴ������ſ�
			return 1;	//���ذ������±�־
		}
	}
	return 0;	
}
void BUZZER_ON()//PE4
{
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET);
}
void BUZZER_OFF()
{
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_SET);
}
void LASER_ON()//PE5
{
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_SET);
}
void LASER_OFF()
{
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_RESET);
}
void STATE(uint8_t state)//PF5,PC7,PC9//��������״ָ̬ʾ
{
	switch(state)
	{
		case 0X01://���
		{  
		   HAL_GPIO_WritePin(GPIOF, GPIO_PIN_5, GPIO_PIN_SET);
		   HAL_GPIO_WritePin(GPIOF, GPIO_PIN_7, GPIO_PIN_RESET);
		   HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_RESET);
		   break;
		}
		case 0X02://�̵�
		{
		   HAL_GPIO_WritePin(GPIOF, GPIO_PIN_5, GPIO_PIN_RESET);
		   HAL_GPIO_WritePin(GPIOF, GPIO_PIN_7, GPIO_PIN_SET);
		   HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_RESET);
			break;
		}
		case 0X03://����
		{
		   HAL_GPIO_WritePin(GPIOF, GPIO_PIN_5, GPIO_PIN_RESET);
		   HAL_GPIO_WritePin(GPIOF, GPIO_PIN_7, GPIO_PIN_RESET);
		   HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_SET);
		   break;
		}
		default:
			{
		   HAL_GPIO_WritePin(GPIOF, GPIO_PIN_5, GPIO_PIN_RESET);
		   HAL_GPIO_WritePin(GPIOF, GPIO_PIN_7, GPIO_PIN_RESET);
		   HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_RESET);
		    }
	}
}
void test()
{
    uint8_t i;
	for ( i = 0; i < 100 ; i++)
	{
		LED_TOGGLE();
		BUZZER_OFF();
		LASER_ON();
		STATE(red);
		HAL_Delay(1000);
		
		LED_TOGGLE(); 
		BUZZER_ON();
		LASER_OFF();
		STATE(green);
		HAL_Delay(1000);

		LED_TOGGLE();
		BUZZER_OFF();
		LASER_ON();
		STATE(blue);
		HAL_Delay(1000);
	}
}

uint16_t abs(int16_t x)
{
	return (x < 0) ? -x : x;
}