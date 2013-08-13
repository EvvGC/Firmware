/*
 * 	systick.c
 *
 *	Created on: Aug 1, 2013
 *		Author: ala42
 */

#include "stm32f10x_tim.h"

static volatile uint32_t sysTickMillis = 0;
static uint32_t sysTickPerUs=72;

static inline int systick_check_underflow(void) 
{
    return SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk;
}

///////////////////////////////////////////////////////////////////////////////
// System Time in Microseconds
///////////////////////////////////////////////////////////////////////////////

unsigned int micros(void)
{
    uint32_t cycle, timeMs;
	do {
		timeMs = sysTickMillis;
		cycle = SysTick->VAL;
        asm volatile("nop");
        asm volatile("nop");
    } while (timeMs != sysTickMillis);

	if(systick_check_underflow()) {
		timeMs++;
		cycle = SysTick->VAL;
	}

    return (timeMs * 1000) + (SysTick->LOAD + 1 - cycle) / sysTickPerUs;
}

///////////////////////////////////////////////////////////////////////////////
// System Time in Milliseconds
///////////////////////////////////////////////////////////////////////////////

unsigned int millis(void)
{
    return sysTickMillis;
}


///////////////////////////////////////////////////////////////////////////////
// SysTick
///////////////////////////////////////////////////////////////////////////////
static void (*systickUserCallback)(void);

void SysTick_Handler(void)
{
    __disable_irq();
	systick_check_underflow();
    sysTickMillis++;
    __enable_irq();

    if (systickUserCallback) {
        systickUserCallback();
    }	
}


void SysTickAttachCallback(void (*callback)(void)) 
{
    systickUserCallback = callback;
}


void InitSysTick(void)
{
	sysTickPerUs = SystemCoreClock / 1000000;
    SysTick_Config(SystemCoreClock / 1000);
}
