/*
 *  systick.c
 *
 *  Created on: Aug 1, 2013
 *      Author: ala42
 */

/*
    Original work Copyright (c) 2012 [Evaldis - RCG user name]
    Modified work Copyright 2012 Alan K. Adamson

    This file is part of EvvGC.

    EvvGC is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    EvvGC is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with EvvGC.  If not, see <http://www.gnu.org/licenses/>.

*/

#include "stm32f10x.h"

static volatile uint32_t sysTickMillis = 0;
static uint32_t sysTickPerUs = 72;

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

    do
    {
        timeMs = sysTickMillis;
        cycle = SysTick->VAL;
        asm volatile("nop");
        asm volatile("nop");
    }
    while (timeMs != sysTickMillis);

    if (systick_check_underflow())
    {
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

    if (systickUserCallback)
    {
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
    //NVIC_SetPriority(SysTick_IRQn, 0);//set systick interrupt priority, 0 is the highest for all
}
