#ifndef __IREIGHT_MODEL_H
#define __IREIGHT_MODEL_H

#include <Arduino.h>

#ifdef __cplusplus
extern "C" {
#endif

#define IR_Num 8
extern uint8_t IR_Data_number[IR_Num];
extern uint16_t IR_Data_Anglo[IR_Num];
extern uint8_t g_new_package_flag;

void serial_init(void);
void recv_data(void);
void SET_Eight_Mode(uint8_t adjust, uint8_t ao, uint8_t num);
void Data_Deal(char RXdata);
void Deal_Usart_Data(void);
void Deal_Usart_AData(void);

#ifdef __cplusplus
}
#endif

#endif