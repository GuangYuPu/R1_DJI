#include "RS485.h"
#include "usart.h"

float RS485_angle;
uint8_t RS485_ReceiveData[20];

void RS485_init()
{
    HAL_UART_Receive_DMA(&huart3,RS485_ReceiveData,20);
}

void RS485_decode()
{
    for(int i = 0 ; i < 20 ; ++i)
    {
        if(
           (RS485_ReceiveData[i] == 0xAB)
        && (RS485_ReceiveData[i+1] == 0xCD)
        && (RS485_ReceiveData[i+2] == 0x05)
        && (RS485_ReceiveData[i+7] == RS485_ReceiveData[i+2]+RS485_ReceiveData[i+3]+RS485_ReceiveData[i+4]+RS485_ReceiveData[i+5]+RS485_ReceiveData[i+6])
        && (RS485_ReceiveData[i+9] == 0x3D)
        )
        {
            RS485_angle = ((((float)(RS485_ReceiveData[i+3]))*256+((float)(RS485_ReceiveData[i+3]))/16384))*360;
        }
    }
}