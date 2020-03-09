#ifndef _WORK_H
#define _WORK_H

#include "stm32f10x.h"

void LED_IO_INIT(void);
void TIME4_init(u16 arr,u16 psc);
void TIM4_IRQHandler(void);
void TIME4_PWM_init(uint32_t fre)  ;                    //∂® ±∆˜≈‰÷√
void pcm_data_tim3_reable(void);

#endif

