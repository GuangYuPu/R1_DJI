#include "RS485.h"
#include "usart.h"

float RS485_angle,data3,data4;
uint8_t RS485_ReceiveData[20];
uint16_t rs_decode = 0;
void RS485_init()
{
    HAL_UART_Receive_DMA(&huart7,RS485_ReceiveData,20);
}

void RS485_decode()
{
    for(int i = 0 ; i < 20 ; ++i)
    {
        if(
           (RS485_ReceiveData[i] == 0xAB)
        && (RS485_ReceiveData[i+1] == 0xCD)
//        && (RS485_ReceiveData[i+2] == 0x05)
        )
        {          
					rs_decode = 1;
//					data3 = RS485_ReceiveData[i+3];
//					data4 = RS485_ReceiveData[i+4];
//					RS485_angle = ((data3*256+data4)/16384)*360;
				  rs_decode = ((int16_t)RS485_ReceiveData[i+3]<<8)|((uint16_t)RS485_ReceiveData[i+4]);
					RS485_angle = ((float)(rs_decode))*360/16384;
				}
				break;
    }
}