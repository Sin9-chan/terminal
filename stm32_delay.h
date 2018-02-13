#ifndef __STM32_DELAY_H
#define __STM32_DELAY_H
#endif

#include "stm32f10x.h"


void DelayInit(void);
void _delay_us(uint32_t us);
void _delay_ms(uint32_t ms);
