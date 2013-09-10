/*
 *  reboot.c
 *
 *  Created on: Aug 28, 2013
 *      Author: ala42
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

