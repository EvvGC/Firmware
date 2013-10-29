/*
 *  reboot.c
 *
 *  Created on: Aug 28, 2013
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

#include "reboot.h"
#include "stm32f10x.h"
#include "core_cm3.h"
#define AIRCR_RESET         0x05FA0000
#define AIRCR_RESET_REQ     (AIRCR_RESET | 0x04);


#define USER_CODE_RAM               (0x20000C00)
#define BOOT_LOADER_MAGIC_ADDR      ((unsigned long*)(USER_CODE_RAM-4))
#define START_BOOT_LOADER_MAGIC     (0x4AFC6BB2)
#define START_MAIN_MAGIC            (0x4AFC6BB3)
#define STAY_IN_BOOTLOADER_MAGIC    (0x4AFC6BB4)


void BKPInit(void)
{
    /* Enable clock for Power interface */
    RCC->APB1ENR |= RCC_APB1ENR_BKPEN | RCC_APB1ENR_PWREN;
}

unsigned long BKPRead(void)
{
    /* Enable access to RTC/BKP registers */
    PWR->CR |= PWR_CR_DBP;

    unsigned long val = BKP->DR41 | (BKP->DR42 << 16);

    /* Disable access to the RTC/BKP registers */
    PWR->CR &= ~PWR_CR_DBP;

    return val;
}

void BKPWrite(unsigned long val)
{
    /* Enable access to RTC/BKP registers */
    PWR->CR |= PWR_CR_DBP;

    BKP->DR41 = val & 0xffff;
    BKP->DR42 = val >> 16;

    /* Disable access to the RTC/BKP registers */
    PWR->CR &= ~PWR_CR_DBP;
}

#if 0
void BKPTest(void)
{
    print("BKPTest\r\n");

    BKPInit();

    print("BKP DR42 %08X\r\n", BKPRead());
    BKPWrite(0x12345678);
    print("BKP DR42 %08X\r\n", BKPRead());

    print("BKPTest done\r\n");
}
#endif


void bootloader(void)
{
    extern uint32_t g_pfnVectors;

    if ((uint32_t)&g_pfnVectors == 0x08000000)
    {
        // there is no bootloader at the bottom of flash memory
        return;
    }

    BKPInit();
    BKPWrite(STAY_IN_BOOTLOADER_MAGIC);
    reboot();
}

void reboot(void)
{
    //SCB_Type* rSCB = (SCB_Type *) SCB_BASE;

    /* Reset  */
    SCB->AIRCR = AIRCR_RESET_REQ;

    /*  should never get here */
    while (1)
    {
        asm volatile("nop");
    }
}

