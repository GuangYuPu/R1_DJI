#ifndef _NRF_COM_H__
#define _NRF_COM_H__

#include "main.h"

#define HEADER_HIGH 0xff
#define HEADER_LOW 0X99

#define HEADER_LENGTH 2
#define PAY_LOAD_LENGTH 10
#define BAG_LENGTH (HEADER_LENGTH + PAY_LOAD_LENGTH)

typedef union{
    uint8_t raw[BAG_LENGTH];
    struct 
    {
        uint8_t header[HEADER_LENGTH];
        union
        {
            uint8_t pay_load[PAY_LOAD_LENGTH];
            struct 
            {
              int16_t ch0;
							int16_t ch1;
							int16_t ch2;
							int16_t ch3;
							int8_t left;
							int8_t right;
            }__attribute__((packed));
        };
    }__attribute__((packed));
}__attribute__((packed)) nrfBag_t;

extern nrfBag_t nrfDataBag;
extern uint8_t nrfReceive[BAG_LENGTH*2];
extern int16_t ch0, ch1, ch2, ch3; 
extern int8_t left,right;

void nrf_Transmit_init();
void send();
void nrf_receive_init();
void nrf_decode();

#endif