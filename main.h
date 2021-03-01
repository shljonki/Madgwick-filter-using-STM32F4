/* main.h */
#ifndef MAIN_H
#define MAIN_H

#include <stm32f4xx.h> // common stuff
#include <stm32f4xx_gpio.h> // gpio control
#include <stm32f4xx_rcc.h> // reset anc clocking

#include <gpio.h>
#include "USART.h"
#include "tm_stm32f4_i2c.h"
#include "tm_stm32f4_gpio.h"
#include "defines.h"
#include "ICM_20948.h"
#include "AHRS.h"

#define DEG_TO_RAD (float)((float)3.14159265358979/(float)180.0)
void USART1_Init(void); // init USART1 peripheral
void USART1_SendChar(char c); // blocking send character
int USART1_Dequeue(char* c); // pop character from receive FIFO
void USART1_SendFloat(float x);
void TM_DELAY_Init(void);
float *getQ(void);
void f(float *kappa, float *sigma, float *qkoso, float *qzos, float *mag_bias);



#endif
