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
#include "main.h"
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

int pwm_init = 799;

float speed = 0;
float pitch = 0;
float yaw = 0;
float fetch = 0;

float pianhang_state_zone0 = 55.649;
float yangjiao_state_zone0 = -24;
float sheqiu_servo_zone0 = 383;
float mocalun_speed_zone0 = 5000;

float pianhang_state_zone1 = 0;
float yangjiao_state_zone1 = 0;
float sheqiu_servo_zone1 = 395;
float mocalun_speed_zone1 = 5000;

float pianhang_state_zone2 = 0;
float yangjiao_state_zone2 = 0;
float sheqiu_servo_zone2 = 395;
float mocalun_speed_zone2 = 5000;

float pianhang_state_zone3 = 0;
float yangjiao_state_zone3 = 0;
float sheqiu_servo_zone3 = 395;
float mocalun_speed_zone3 = 5000;

float pianhang_state_zone4 = 0;
float yangjiao_state_zone4 = 0;
float sheqiu_servo_zone4 = 395;
float mocalun_speed_zone4 = 5000;

float pianhang_state_zone5 = 0;
float yangjiao_state_zone5 = 0;
float sheqiu_servo_zone5 = 395;
float mocalun_speed_zone5 = 5000;

float pianhang_state_zone6 = 0;
float yangjiao_state_zone6 = 0;
float sheqiu_servo_zone6 = 395;
float mocalun_speed_zone6 = 5000;

float pianhang_state_zone7 = 0;
float yangjiao_state_zone7 = 0;
float sheqiu_servo_zone7 = 395;
float mocalun_speed_zone7 = 5000;

float pianhang_state_zone8 = 0;
float yangjiao_state_zone8 = 0;
float sheqiu_servo_zone8 = 395;
float mocalun_speed_zone8 = 5000;

float pianhang_state_zone9 = 0;
float yangjiao_state_zone9 = 0;
float sheqiu_servo_zone9 = 395;
float mocalun_speed_zone9 = 5000;

float pianhang_state_zone10 = 0;
float yangjiao_state_zone10 = 0;
float sheqiu_servo_zone10 = 395;
float mocalun_speed_zone10 = 5000;

float pianhang_state_zone11 = 0;
float yangjiao_state_zone11 = 0;
float sheqiu_servo_zone11 = 395;
float mocalun_speed_zone11 = 5000;

float pianhang_state_zone12 = 0;
float yangjiao_state_zone12 = 0;
float sheqiu_servo_zone12 = 395;
float mocalun_speed_zone12 = 5000;

float fetch_state = 0;
float mocalun_state = 0;

uint32_t time = 0;
uint32_t enter_time = 0;

//parameters for sheqiu
uint32_t sj_time = 300;
uint32_t zz_time = 1000;
uint32_t waiting_time = 1000;
float close_speed = -100;
float open_speed = 60;

//parameter for quqiu
uint32_t sj_time_1 = 1500;
uint32_t zz_time_1 = 1000;
uint32_t waiting_time_1 = 800;
float close_speed_1 = -150;
float open_speed_1 = 180;

float open_pos = 22;
float close_pos = 0;

uint32_t rs_decode_strart = 0;

char flag = 0;//标志是否进入状�?? 
/*状�?�机变换�????
state = 0 状�??0 遥控器控制中间状�???? 初始态及其他任何状�?�之间的连接�????
state = 1 状�??1 全自动取�????
state = 2 状�??2 全自动射�???? 并准备取�?
*/
uint32_t state = 0;
uint32_t last_state = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void zz_sj_servo(float ref_rs_decode);
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
	__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, pwm_init);
	
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
     	if((Raw_Data.ch2-1024)>100) pitch += 0.05;/*==================================================================*/
			if((Raw_Data.ch2-1024)<-100) pitch -= 0.05;/*==================================================================*/
			if((Raw_Data.ch3-1024)>100) yaw -= 0.1;/*==================================================================*/
			if((Raw_Data.ch3-1024)<-100) yaw += 0.1;/*==================================================================*/
			
			if((Raw_Data.ch0-1024)>100) fetch = 100;/*==================================================================*/
			else if((Raw_Data.ch0-1024)<-100) fetch = -100;/*==================================================================*/
			else fetch = 0;
			
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
			
			if(Raw_Data.right == 1) speed = 3000;
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
			positionServo(fetch_state,&hDJI[4]);
			if(zone == 0)
			{
			positionServo(yangjiao_state_zone0,&hDJI[5]);
			positionServo(pianhang_state_zone0,&hDJI[6]);
			}	
			else if(zone == 1)
			{
			positionServo(yangjiao_state_zone1,&hDJI[5]);
			positionServo(pianhang_state_zone1,&hDJI[6]);
			}	
			else if(zone == 2)
			{
			positionServo(yangjiao_state_zone2,&hDJI[5]);
			positionServo(pianhang_state_zone2,&hDJI[6]);
			}	
      else if(zone == 3)
			{
			positionServo(yangjiao_state_zone3,&hDJI[5]);
			positionServo(pianhang_state_zone3,&hDJI[6]);
			}	
      else if(zone == 4)
			{
			positionServo(yangjiao_state_zone4,&hDJI[5]);
			positionServo(pianhang_state_zone4,&hDJI[6]);
			}	
      else if(zone == 5)
			{
			positionServo(yangjiao_state_zone5,&hDJI[5]);
			positionServo(pianhang_state_zone5,&hDJI[6]);
			}	
      else if(zone == 6)
			{
			positionServo(yangjiao_state_zone6,&hDJI[5]);
			positionServo(pianhang_state_zone6,&hDJI[6]);
			}	
      else if(zone == 7)
			{
			positionServo(yangjiao_state_zone7,&hDJI[5]);
			positionServo(pianhang_state_zone7,&hDJI[6]);
			}	
      else if(zone == 8)
			{
			positionServo(yangjiao_state_zone8,&hDJI[5]);
			positionServo(pianhang_state_zone8,&hDJI[6]);
			}	
      else if(zone == 9)
			{
			positionServo(yangjiao_state_zone9,&hDJI[5]);
			positionServo(pianhang_state_zone9,&hDJI[6]);
			}	
      else if(zone == 10)
			{
			positionServo(yangjiao_state_zone10,&hDJI[5]);
			positionServo(pianhang_state_zone10,&hDJI[6]);
			}	
      else if(zone == 11)
			{
			positionServo(yangjiao_state_zone11,&hDJI[5]);
			positionServo(pianhang_state_zone11,&hDJI[6]);
			}	
      else if(zone == 12)
			{
			positionServo(yangjiao_state_zone12,&hDJI[5]);
			positionServo(pianhang_state_zone12,&hDJI[6]);
			}	
		}
		//执行取球操作
		else if(state == 1){
			speedServo(0,&hDJI[0]);
			speedServo(0,&hDJI[1]);
			speedServo(0,&hDJI[2]);
			speedServo(0,&hDJI[3]);
			positionServo(fetch_state,&hDJI[4]);
			positionServo(0,&hDJI[5]);
			positionServo(0,&hDJI[6]);
			// positionServo(0,&hDJI[7]);
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
void zz_sj_servo(float ref_rs_decode)
{
  //when rsdecode_exp = ref_rs_decode
            if(rs_decode < ref_rs_decode-2)//up
            {
              if(abs(rs_decode - rs_decode_strart)<200 || abs(rs_decode-ref_rs_decode)<200) __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, pwm_init);
              else __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, 999);
            HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_SET);
            }
            else if(rs_decode > ref_rs_decode+2)//down
            {
              if(abs(rs_decode - rs_decode_strart)<200 || abs(rs_decode-ref_rs_decode)<200) __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, 399);
              else __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, 499);
            HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET);
            }
            else//stop
            {
            HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET);
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
          float mocalun_speed = 0;
          float sheqiu_servo = 395; 
          switch (zone)
          {
            case /* constant-expression */0:
            /* code */
            mocalun_speed = mocalun_speed_zone0;
            sheqiu_servo = sheqiu_servo_zone0;
            break;
            case /* constant-expression */1:
            /* code */
            mocalun_speed = mocalun_speed_zone1;
            sheqiu_servo = sheqiu_servo_zone1;
            break;
            case /* constant-expression */2:
            /* code */
            mocalun_speed = mocalun_speed_zone2;
            sheqiu_servo = sheqiu_servo_zone2;
            break;
            case /* constant-expression */3:
            /* code */
            mocalun_speed = mocalun_speed_zone3;
            sheqiu_servo = sheqiu_servo_zone3;
            break;
            case /* constant-expression */4:
            /* code */
            mocalun_speed = mocalun_speed_zone4;
            sheqiu_servo = sheqiu_servo_zone4;
            break;
            case /* constant-expression */5:
            /* code */
            mocalun_speed = mocalun_speed_zone5;
            sheqiu_servo = sheqiu_servo_zone5;
            break;
            case /* constant-expression */6:
            /* code */
            mocalun_speed = mocalun_speed_zone6;
            sheqiu_servo = sheqiu_servo_zone6;
            break;
            case /* constant-expression */7:
            /* code */
            mocalun_speed = mocalun_speed_zone7;
            sheqiu_servo = sheqiu_servo_zone7;
            break;
            case /* constant-expression */8:
            /* code */
            mocalun_speed = mocalun_speed_zone8;
            sheqiu_servo = sheqiu_servo_zone8;
            break;
            case /* constant-expression */9:
            /* code */
            mocalun_speed = mocalun_speed_zone9;
            sheqiu_servo = sheqiu_servo_zone9;
            break;
            case /* constant-expression */10:
            /* code */
            mocalun_speed = mocalun_speed_zone10;
            sheqiu_servo = sheqiu_servo_zone10;
            break;
            case /* constant-expression */11:
            /* code */
            mocalun_speed = mocalun_speed_zone11;
            sheqiu_servo = sheqiu_servo_zone11;
            break;
            case /* constant-expression */12:
            /* code */
            mocalun_speed = mocalun_speed_zone12;
            sheqiu_servo = sheqiu_servo_zone12;
            break;
          default:
            mocalun_speed = 0;
            sheqiu_servo = 395;
            break;
          }
          if((time - enter_time)<(500))
          {
            mocalun_state = 0;
            fetch_state = 0;
            if((time - enter_time)<(1)) rs_decode_strart = rs_decode;
            zz_sj_servo(325);
          }
          else if((time - enter_time)<(zz_time+500))//open mocalun and open the zhuazi
          {
            mocalun_state = mocalun_speed;
            fetch_state = 0;
            HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET);
          }
          else if((time - enter_time)<(zz_time+sj_time+500))//up shenjiang
          {
            mocalun_state = mocalun_speed;
            fetch_state = 0;
            if((time - enter_time)<(zz_time+500+1)) rs_decode_strart = rs_decode;
            zz_sj_servo(sheqiu_servo);
          }
          else if((time - enter_time)<(zz_time+sj_time+waiting_time+500))//wait for shooting
          {
            mocalun_state = mocalun_speed;
            fetch_state = 0;
            HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET);
          }
          else if((time - enter_time)<(zz_time+sj_time+waiting_time+zz_time+500))//down shenjiang and open the zhuazi
          {
            mocalun_state = 0;
            fetch_state = 0;
            if((time - enter_time)<(zz_time+sj_time+waiting_time+1+500)) rs_decode_strart = rs_decode;
            zz_sj_servo(270);
          }
					else if((time - enter_time)<(zz_time+sj_time+waiting_time+zz_time+500+500))//down shenjiang and open the zhuazi
          {
            mocalun_state = 0;
            fetch_state = open_pos;
            HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET);
          }
          else{
						fetch_state = open_pos;
            last_state = state;
            state = 0;
          }
			    }
		
    else if(state == 1)
					{
          if((time - enter_time)<(500))
          {
            fetch_state = open_pos;
            if((time - enter_time)<(1)) rs_decode_strart = rs_decode;
            zz_sj_servo(185);
          }
          else if((time - enter_time)<(zz_time_1+500))//close zhuazi
          {
            fetch_state = close_pos;
            HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET);
          }
          else if((time - enter_time)<(zz_time_1+sj_time_1+500))//up shenjiang
          {
            fetch_state = close_pos;
            if((time - enter_time)<(zz_time_1+1+500)) rs_decode_strart = rs_decode;
            zz_sj_servo(325);
          }
          else{
						fetch_state = close_pos;
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
