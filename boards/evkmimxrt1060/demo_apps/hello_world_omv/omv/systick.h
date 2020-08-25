#ifndef __TIMER_H__
#define __TIMER_H__
#include "stdlib.h"
#include "stddef.h"
#include "stdint.h"
#include "fsl_common.h"
#define HAL_GetTick get_tick_ms
extern volatile uint32_t msTick;
size_t get_tick_ms();
size_t get_tick_us();
void delay_us(size_t us);
void systick_sleep(size_t ms); 
void init_timer();
#endif