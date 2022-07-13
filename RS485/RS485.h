#ifndef _RS485_H__
#define _RS485_H__

#include "main.h"

void RS485_init();
void RS485_decode();

extern float RS485_angle,data3,data4;
extern uint8_t RS485_ReceiveData[20];
extern uint16_t rs_decode;
extern int debug;

#endif