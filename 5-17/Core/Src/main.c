/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "memorymap.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd_model.h"
#include "PID.h"
#include "stdio.h"
#include "show.h"
#include "control.h"
#include "servo.h"
#include "Usart.h"
#include "hardware.h"
#include "cascade_control.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RX_BUFFER_SIZE 128
#define TIMER_MAX_COUNT 65535 // 定时器最大计数值（16 位定时器）
#define CAMERA_WIDTH 160  
#define CAMERA_HEIGHT 120
#define MIN_ANGLE_X 0.0f  
#define MAX_ANGLE_X 180.0f  
#define MIN_ANGLE_Y 0.0f  
#define MAX_ANGLE_Y 180.0f  
#define PULSES_PER_REV  (13 * 4 * 28) // 编码器线数13，四倍频，减速比28:1
#define TIME_INTERVAL  0.01f         // 定时器中断时间间隔（单位：秒）
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t received_1;
uint8_t received_2;
uint8_t received_3;
uint8_t received_4;
uint8_t received_5;
uint8_t received_6;
uint8_t received_7;
uint8_t received_8;	
uint8_t key_value;
uint8_t rx_buffer[RX_BUFFER_SIZE];
uint8_t rx_len = 0;

uint8_t Send_Data[8]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}; //发送数据数组
float wheel_circumference = 3.14159f * 0.065f; // 单位：米
int32_t position_TIM3 = 0; // 编码器计数位置
int32_t position_TIM4 = 0; // 编码器计数位置
int32_t prev_count_TIM3 = 0;
int32_t prev_count_TIM4 = 0;
float RPM_TIM3 = 0.0f;
float RPM_TIM4 = 0.0f;
float position_angle_TIM3 = 0.0f; // 电机1位置累计角度
float position_angle_TIM4 = 0.0f; // 电机2位置累计角度
float speed_TIM3 = 0 ;// 电机1线速度
float speed_TIM4 = 0 ;// 电机2线速度
float s_TIM3 = 0 ; // 电机1位移
float s_TIM4 = 0 ; // 电机2位移

PID_Controller panPID;  
PID_Controller tiltPID;
CascadePID pid_TIM3 = {0}; // 创建串级PID结构体变量
CascadePID pid_TIM4 = {0}; 

float target_X = CAMERA_WIDTH / 2;  
float target_Y = CAMERA_HEIGHT / 2; 
float x,y;
float servo_rotation_value=90.0;
float servo_pitch_value=90.0;
uint8_t servo_rotation_direction=1;
uint8_t servo_pitch_direction=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_SPI6_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_TIM23_Init();
  MX_UART4_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  SPI_LCD_Init();	            //LCD初始化
  TIM_Init();                //TIM初始化
  USART_Init();              //串口初始化
  //test();                    //硬件测试函数
  //example();             //电机测试函数
  //USART2_Send_Data(Send_Data,8);      //发送数据函数(USART2)

  // 初始化内环参数：比例系数10，积分系数0，微分系数0，最大积分0，最大输出1000
  PID_Init_control(&pid_TIM3.inner, 0.0f, 0.0f, 0.0f,  125, 125);
  // 初始化外环参数：比例系数5，积分系数0，微分系数5，最大积分0，最大输出100
  PID_Init_control(&pid_TIM3.outer, 0.0f, 0.0f, 0.0f,  999, 999);
  // 初始化内环参数：比例系数10，积分系数0，微分系数0，最大积分0，最大输出1000
  PID_Init_control(&pid_TIM4.inner, 0.0f, 0.0f, 0.0f,  125, 125);
  // 初始化外环参数：比例系数5，积分系数0，微分系数5，最大积分0，最大输出100
  PID_Init_control(&pid_TIM3.outer, 0.0f, 0.0f, 0.0f,  999, 999);


  PID_Init(&panPID,  0.01f, 0.0f, 0.003f);       
	PID_Init(&tiltPID, 0.01f, 0.0f, 0.003f);
	PID_SetSetpoint(&panPID,90);     // X设置目标值
	PID_SetSetpoint(&tiltPID,90);    //Y设置目标值
  
  
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
   LCD_Show();
   float outerTarget_TIM3 = getTargetPosition(&htim3,0); // 获取外环目标值
   float outerFeedback_TIM3 = getFeedbackPosition(&htim3); // 获取外环反馈值
   float innerFeedback_TIM3 = getFeedbackSpeed(&htim3); // 获取内环反馈值

   float outerTarget_TIM4  = getTargetPosition(&htim4,0); //取外环目标值
   float outerFeedback_TIM4 = getFeedbackPosition(&htim4); // 获取外环反馈值
   float innerFeedback_TIM4 = getFeedbackSpeed(&htim4);// 获取内环反馈值
   
   target_X=received_1;
	 target_Y=received_2;	

   x=PID_Compute(&panPID, target_X);  
   y=PID_Compute(&tiltPID, target_Y);
   
   PID_CascadeCalc(&pid_TIM3, outerTarget_TIM3, outerFeedback_TIM3, innerFeedback_TIM3); // 进行PID计算
   PID_CascadeCalc(&pid_TIM4, outerTarget_TIM4, outerFeedback_TIM4, innerFeedback_TIM4); // 进行PID计算
   setActuatorOutput(&htim3,pid_TIM3.output);
   setActuatorOutput(&htim4,pid_TIM4.output);

   if (!servo_rotation_direction) 
	{ 
        x = -x;  
    }  
    if (!servo_pitch_direction) 
	{ 
        y = -y;  
    }  
    if (MIN_ANGLE_X < servo_rotation_value + x && servo_rotation_value + x < MAX_ANGLE_X) 
	{  
        servo_rotation_value += x;  	
		    SendServo_X_PWM(angle_to_pulse(servo_rotation_value) );
  }  
    if (MIN_ANGLE_Y < servo_pitch_value + y && servo_pitch_value + y < MAX_ANGLE_Y) 
	{ 
        servo_pitch_value += y;
		    SendServo_Y_PWM(angle_to_pulse(servo_pitch_value) );
  }
  if(rx_buffer[0]==0x01)
  {BUZZER_ON();}
   HAL_Delay(5);






  }






  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 44;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) // 10ms//TIM4为左轮TIM3为右轮
{
    if (htim->Instance == TIM2) 
    {
        // 读取当前计数值
        int16_t current_count_TIM3 = (int16_t)TIM3->CNT;
        int16_t current_count_TIM4 = (int16_t)TIM4->CNT;
        // 计算计数增量
        int16_t delta_TIM3 = current_count_TIM3 - prev_count_TIM3;
        int16_t delta_TIM4 = current_count_TIM4 - prev_count_TIM4;
        if (delta_TIM3 > (TIMER_MAX_COUNT / 2)) 
        {                                           
            delta_TIM3 -= TIMER_MAX_COUNT + 1;// 发生下溢
        } 
        else if (delta_TIM3 < -(TIMER_MAX_COUNT / 2)) 
        {
            delta_TIM3 += TIMER_MAX_COUNT + 1; // 发生上溢
        }
          if (delta_TIM4 > (TIMER_MAX_COUNT / 2)) 
        {                                           
            delta_TIM4 -= TIMER_MAX_COUNT + 1;// 发生下溢
        } 
        else if (delta_TIM4 < -(TIMER_MAX_COUNT / 2)) 
        {
            delta_TIM4 += TIMER_MAX_COUNT + 1; // 发生上溢
        }
        // 计算当前位置（单位：脉冲数）
        position_TIM3 += delta_TIM3;
        position_TIM4 += delta_TIM4;
        // 更新上一次计数值
        prev_count_TIM3 = current_count_TIM3;
        prev_count_TIM4 = current_count_TIM4;
        // 计算转速（RPM）
        RPM_TIM3 = (delta_TIM3 / (float)PULSES_PER_REV) * (60.0f / TIME_INTERVAL);
        RPM_TIM4 = (delta_TIM4 / (float)PULSES_PER_REV) * (60.0f / TIME_INTERVAL);
        // 转换为实际位置（单位：角度）
        position_angle_TIM3 = (position_TIM3 /(float)PULSES_PER_REV) * 360.0f; // 单位：度
        position_angle_TIM4 = (position_TIM4 /(float)PULSES_PER_REV) * 360.0f; // 单位：度
        // 计算位移变化量（单位：米）
        float delta_s_TIM3 = (delta_TIM3 / (float)PULSES_PER_REV) * wheel_circumference;
        float delta_s_TIM4 = (delta_TIM4 / (float)PULSES_PER_REV) * wheel_circumference;
        // 累计位移量（单位：米）
        s_TIM3 = (position_TIM3 / (float)PULSES_PER_REV) * wheel_circumference;
        s_TIM4 = (position_TIM4 / (float)PULSES_PER_REV) * wheel_circumference;
        // 计算线速度（单位：米/秒）
        speed_TIM3 = delta_s_TIM3 / TIME_INTERVAL;
        speed_TIM4 = delta_s_TIM4 / TIME_INTERVAL;
        // 输出当前位置
        printf("Position TIM3: %ld pulses, %.2f degrees\r\n", position_TIM3, position_angle_TIM3);
        printf("Position TIM4: %ld pulses, %.2f degrees\r\n", position_TIM4, position_angle_TIM4);
        // 输出转速
        // printf("RPM_TIM3: %.2f, RPM_TIM4: %.2f\r\n", RPM_TIM3, RPM_TIM4);
        // 输出位移
        printf("Displacement TIM3: %.4f m, Displacement TIM4: %.4f m\r\n", s_TIM3, s_TIM4);
        // 输出速度
        printf("Speed TIM3: %.4f m/s, Speed TIM4: %.4f m/s\r\n", speed_TIM3, speed_TIM4);
        
    }
}
/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
