#include "lcd_model.h"
#include "show.h"

void LCD_Show(void)//240*280
{
	LCD_SetBackColor(LCD_BLACK); 			
	LCD_Clear(); 							
	LCD_SetColor(LIGHT_GREEN);					
	
	LCD_SetAsciiFont(&ASCII_Font20);
	LCD_DisplayString(20,10,"received_1:");
    LCD_DisplayString(20,30,"received_2:");		
	
	LCD_SetAsciiFont(&ASCII_Font20);
	LCD_DisplayNumber(130,10,received_1,3);
	LCD_DisplayNumber(130,30,received_2,3);
	
	LCD_SetAsciiFont(&ASCII_Font20);
	LCD_DisplayString(20,50,"received_3:");
    LCD_DisplayString(20,70,"received_4:");	

	LCD_SetAsciiFont(&ASCII_Font20);
	LCD_DisplayNumber(130,50,received_3,3);
	LCD_DisplayNumber(130,70,received_4,3);
	
	LCD_SetAsciiFont(&ASCII_Font20);
	LCD_DisplayString(20,90,"received_5:");
    LCD_DisplayString(20,110,"received_6:");	

	LCD_SetAsciiFont(&ASCII_Font20);
	LCD_DisplayNumber(130,90,received_5,3);
	LCD_DisplayNumber(130,110,received_6,3);
	
	LCD_SetAsciiFont(&ASCII_Font20);
	LCD_DisplayString(20,130,"received_7:");
    LCD_DisplayString(20,150,"received_8:");	

	LCD_SetAsciiFont(&ASCII_Font20);
	LCD_DisplayNumber(130,130,received_7,3);
	LCD_DisplayNumber(130,150,received_8,3);
	
	// LCD_SetColor(LIGHT_RED);		
	// LCD_DisplayString(20,90,"key_value:");
	// LCD_DisplayNumber(120,90,key_value,3);
	// LCD_SetColor(LIGHT_BLUE);
	// LCD_DisplayString(20,110,"Designed By Tupls");
	// LCD_SetColor(LIGHT_GREEN);
	// LCD_DisplayString(20,130,"NUIST");
	HAL_Delay(5);
	
}