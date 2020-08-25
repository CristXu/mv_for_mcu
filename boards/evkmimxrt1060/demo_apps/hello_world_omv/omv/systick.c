#include "systick.h"
#include "core_cm7.h"
#define SYSTICK_MS (SystemCoreClock/1000U)
#define SYSTICK_US (SystemCoreClock/1000000U)
volatile uint32_t msTick;
void SysTick_Handler(){
	++msTick;
}
size_t get_tick_ms(){
	return msTick;
}
size_t get_tick_us(){
	size_t us = (SYSTICK_MS - SysTick->VAL) / SYSTICK_US;
	us += msTick * 1000;
	return us;	
}
void delay_us(size_t us){
	size_t us_1 = get_tick_us();
	while((get_tick_us() - us_1) < us);
}
void systick_sleep(size_t ms){
	size_t ms_1 = get_tick_ms();
	while((get_tick_ms() - ms_1) < ms);
}
void init_timer(){
	SysTick_Config(SYSTICK_MS);
}