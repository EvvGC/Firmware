/*
 *  utils.c
 *
 *  Created on: Jun 25, 2013
 *      Author: Denis caat
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

#include <math.h>
#include "utils.h"
#include "stm32f10x.h"
#include "comio.h"
#include "pins.h"

void LEDon(void)
{
    GPIO_SetBits(LED1_PORT, LED1_PIN); //LED on
}

void LEDoff(void)
{
    GPIO_ResetBits(LED1_PORT, LED1_PIN); //LED off
}

void LEDtoggle(void)
{
    __disable_irq();
    GPIO_ToggleBits(LED1_PORT, LED1_PIN);
    __enable_irq();
}

void DEBUG_LEDon(void)
{
    GPIO_SetBits(LED2_PORT, LED2_PIN); //LED on
}

void DEBUG_LEDoff(void)
{
    GPIO_ResetBits(LED2_PORT, LED2_PIN); //LED off
}

void DEBUG_LEDtoggle(void)
{
    __disable_irq();
    GPIO_ToggleBits(LED2_PORT, LED2_PIN);
    __enable_irq();
}

void Blink(void)
{
    DEBUG_PutChar('B');

    LEDon();            //blinking led
    Delay_ms(200);
    LEDoff();
    Delay_ms(200);
}

#define STM32_DELAY_US_MULT         12

void Delay_us(unsigned int us)
{
    us *= STM32_DELAY_US_MULT;

    /* fudge for function call overhead  */
    //us--;
    asm volatile("   mov r0, %[us]          \n\t"
                 "1: subs r0, #1            \n\t"
                 "   bhi 1b                 \n\t"
                 :
                 : [us] "r"(us)
                 : "r0");
}

void Delay_ms(unsigned int ms)
{
    Delay_us(1000 * ms);
}

float Rad2Deg(float x)
{
    return x * (180.0F / M_PI);
}

float Deg2Rad(float x)
{
    return x * (M_PI / 180.0F);
}

float Round(float x)
{
    if (x >= 0)
    {
        return x + 0.5F;
    }
    else
    {
        return x - 0.5F;
    }
}
