#include "nrf_com.h"
#include "main.h"
#include "usart.h"

nrfBag_t nrfDataBag;
uint8_t nrfReceive[BAG_LENGTH*2];
int16_t ch0 = 1024;
int16_t ch1 = 1024;
int16_t ch2 = 1024;
int16_t ch3 = 1024; 
int8_t left = 3;
int8_t right = 3;

//发送单片机初始化
void nrf_Transmit_init()
{
    nrfDataBag.header[0] = HEADER_HIGH;
    nrfDataBag.header[1] = HEADER_LOW;
}

//发送单片机发送函数，循环调用
void send()
{
    HAL_UART_Transmit(&huart6,nrfDataBag.raw,BAG_LENGTH,50);
} 

//接收单片机初始化
void nrf_receive_init()
{
		for(int i=0;i<50;i++)
	{
		//Delay
	}
    HAL_UART_Receive_DMA(&huart6,nrfReceive,BAG_LENGTH*2);
}

//在回调函数中进行解码
void nrf_decode()
{
      for(int i = 0 ; i < BAG_LENGTH*2 ; ++i)
      {
          nrfBag_t tempBag = *(nrfBag_t*)(void*)((&nrfReceive[i]));
          if(
              tempBag.header[0] == HEADER_HIGH    &&
              tempBag.header[1] == HEADER_LOW  
          ){
              ch0 = tempBag.ch0;
              ch1 = tempBag.ch1;
              ch2 = tempBag.ch2;
						  ch3 = tempBag.ch3;
						left = tempBag.left;
						right = tempBag.right;
              break;          
          }
      }
}

//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//    if(huart->Instance == huart4.Instance)
//    {
//        nrf_decode();
//    }
//}