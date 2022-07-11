/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Caculate.h"
#include "DJI.h"
#include "wtr_can.h"
#include "wtr_uart.h"
#include "nrf_com.h"
#include "RS485.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
int ifRecv_dji = 0;
int ifRecv_RS485 = 0;
int ifRecv_DiPan = 0;

float speed = 0;
float pitch = 0;
float yaw = 0;
float pitch_state = 0;
float yaw_state = 0;
float fetch = 0;

float fetch_state = 0;
float mocalun_state = 0;
uint32_t time = 0;
uint32_t enter_time = 0;

uint32_t sj_time = 200;
uint32_t zz_time = 1000;
uint32_t waiting_time = 1000;
float close_speed = -100;
float open_speed = 180;

uint32_t sj_time_1 = 1000;
uint32_t zz_time_1 = 500;
uint32_t waiting_time_1 = 1000;
float close_speed_1 = -100;
float open_speed_1 = 180;

char flag = 0;//标志是否进入状�?? 
/*状�?�机变换�????
state = 0 状�??0 遥控器控制中间状�???? 初始态及其他任何状�?�之间的连接�????
state = 1 状�??1 全自动取�????
state = 2 状�??2 全自动射�???? 并准备取�?
*/
uint32_t state = 0;
uint32_t last_state = 3;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
  MX_CAN1_Init();
  MX_USART1_UART_Init();
  MX_USART6_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART3_UART_Init();
  MX_UART8_Init();
  /* USER CODE BEGIN 2 */
 CANFilterInit(&hcan1);
	
	hDJI[0].motorType = M3508;
	hDJI[1].motorType = M3508;
	hDJI[2].motorType = M3508;
	hDJI[3].motorType = M3508;
	hDJI[4].motorType = M3508;
	hDJI[5].motorType = M3508;
	hDJI[6].motorType = M2006;
	hDJI[7].motorType = M3508;
	
	DJI_Init();
	
  HAL_UART_Receive_DMA(&huart1,JoyStickReceiveData,18);
	
	nrf_receive_init();
	
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_Start_IT(&htim3);
	
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
	__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, 799);
	
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_12,1);
  
  RS485_init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
  {
		while(ifRecv_dji==0)
    {
    }
		if(state == 0)
		{
     	if((Raw_Data.ch2-1024)>100) pitch += 0.25;/*==================================================================*/
			if((Raw_Data.ch2-1024)<-100) pitch -= 0.25;/*==================================================================*/
			if((Raw_Data.ch3-1024)>100) yaw -= 0.15;/*==================================================================*/
			if((Raw_Data.ch3-1024)<-100) yaw += 0.15;/*==================================================================*/
			
			if((Raw_Data.ch0-1024)>100) fetch = 100;/*==================================================================*/
			else if((Raw_Data.ch0-1024)<-100) fetch = -100;/*==================================================================*/
			else 	fetch = 0;
			
			if((Raw_Data.ch1-1024)>100) 
			{
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_SET);
			}
			else if((Raw_Data.ch1-1024)<-100)
			{
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET);
			}
			else
			{
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET);
			}
			
			positionServo(yaw,&hDJI[5]);
			positionServo(pitch,&hDJI[6]);
			speedServo(fetch,&hDJI[4]);
			
			if(Raw_Data.right == 1) speed = 5000;
			else speed = 0;
			
			speedServo(speed,&hDJI[0]);
			speedServo(speed,&hDJI[1]);
			speedServo(-speed,&hDJI[2]);
			speedServo(-speed,&hDJI[3]);
		}
    //执行射球
    else if(state == 2){
			speedServo(mocalun_state,&hDJI[0]);
			speedServo(mocalun_state,&hDJI[1]);
			speedServo(-mocalun_state,&hDJI[2]);
			speedServo(-mocalun_state,&hDJI[3]);
			speedServo(fetch_state,&hDJI[4]);
			if(zone == 0)
			{
			positionServo(yaw_state,&hDJI[5]);
			positionServo(pitch_state,&hDJI[6]);
			}	
			if(zone == 1)
			{
			positionServo(yaw_state,&hDJI[5]);
			positionServo(pitch_state,&hDJI[6]);
			}	
			if(zone == 2)
			{
			positionServo(yaw_state,&hDJI[5]);
			positionServo(pitch_state,&hDJI[6]);
			}	
		}
		//执行取球操作
		else if(state == 1){
			speedServo(0,&hDJI[0]);
			speedServo(0,&hDJI[1]);
			speedServo(0,&hDJI[2]);
			speedServo(0,&hDJI[3]);
			speedServo(fetch_state,&hDJI[4]);
			positionServo(0,&hDJI[5]);
			positionServo(0,&hDJI[6]);
			positionServo(0,&hDJI[7]);
		}
		
		CanTransmit_DJI_1234(&hcan1,
                             hDJI[0].speedPID.output,
                             hDJI[1].speedPID.output,
                             hDJI[2].speedPID.output,
                             hDJI[3].speedPID.output);
														 
		CanTransmit_DJI_5678(&hcan1,
                             hDJI[4].speedPID.output,
                             hDJI[5].speedPID.output,
                             hDJI[6].speedPID.output,
                             hDJI[7].speedPID.output);
														 
		
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		HAL_Delay(1);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
    

   if(huart->Instance == huart1.Instance){
       ifRecv_dji = 1;
       UART1Decode();
    }
		
		if(huart->Instance == huart6.Instance)
    {
      ifRecv_DiPan = 1;
      nrf_decode();
    }

    if(huart->Instance == huart8.Instance)
    {
      ifRecv_RS485 = 1;
      RS485_decode();
    }
}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
	/*维护系统时钟系统*/
  if (htim == (&htim2))
	{
		time++;
	}
  /*有限�?*/
	if (htim == (&htim3))
	{	
    if(state == 0 && last_state == 0)
		{
			if(Raw_Data.right == 2 && state == 0)
      {
        enter_time = time;
				state = 1;
			}
      if(Raw_Data.right == 1 && state == 0)
      {
        enter_time = time;
				state = 2;
			}
		}

    
		if(state == 0 && last_state == 1)
		{
      if(Raw_Data.left == 1 && state == 0)
      {
        enter_time = time;
				state = 2;
			}
		}

    
		if(state == 0 && last_state == 2)
		{
      if(Raw_Data.right == 2 && state == 0)
      {
        enter_time = time;
				state = 1;
			}
      
		}
    
        if(state == 2)
					{
          if((time - enter_time)<(zz_time))//open mocalun and open the zhuazi
          {
            mocalun_state = 500;
            fetch_state = 0;
            HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET);
          }
          else if((time - enter_time)<(zz_time+sj_time))//up shenjiang
          {
            mocalun_state = 500;
            fetch_state = 0;
            //when rsdecode_exp = 125
            if(rs_decode < 100)//up
            {
            HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_SET);
            }
            else if(rs_decode > 150)//down
            {
            HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET);
            }
            else//stop
            {
            HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET);
            }
          }
          else if((time - enter_time)<(zz_time+sj_time+waiting_time))//wait for shooting
          {
            mocalun_state = 500;
            fetch_state = 0;
            HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET);
          }
          else if((time - enter_time)<(zz_time+sj_time+waiting_time+sj_time_1))//down shenjiang and open the zhuazi
          {
            mocalun_state = 0;
            fetch_state = open_speed;
            //when rsdecode_exp = 125
            if(rs_decode < 100)
            {
            HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_SET);
            }
            else if(rs_decode > 150)
            {
            HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET);
            }
            else
            {
            HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET);
            }
          }
          else{
						fetch_state = 0;
            last_state = state;
            state = 0;
          }
			    }
		
    else if(state == 1)
					{
          if((time - enter_time)<(zz_time_1))//close zhuazi
          {
            fetch_state = close_speed_1;
            HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET);
          }
          else if((time - enter_time)<(zz_time_1+sj_time_1))//up shenjiang
          {
            fetch_state = 0;
            //when rsdecode_exp = 125
            if(rs_decode < 100)
            {
            HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_SET);
            }
            else if(rs_decode > 150)
            {
            HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET);
            }
            else
            {
            HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET);
            }
          }
          else{
						fetch_state = 0;
            last_state = state;
            state = 0;
            flag = 0;
          }
			    }


	}
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
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
