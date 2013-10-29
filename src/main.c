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

#include <stdint.h>
#include "stm32f10x.h"
#include "main.h"
#include "adc.h"
#include "comio.h"
#include "commhandler.h"
#include "config.h"
#include "fasttrig.h"
#include "engine.h"
#include "gyro.h"
#include "pwm.h"
#include "pins.h"
#include "rc.h"
#include "systick.h"
#include "utils.h"
#include "hw_config.h"
#include "reboot.h"

// echo b > /dev/ttyACM0 && sudo ./dfu-util -v -a 1 -d 1eaf:0003 -D out/STM32Gimbal.USB.bin
// echo b > /dev/cu.usbmodemfa1341 && dfu-util -v -a 1 -d 1eaf:0003 -D STM32Gimbal.USB.bin

static volatile int WatchDogCounter;

void Periph_clock_enable(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
                           RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD |
                           RCC_APB2Periph_GPIOE | RCC_APB2Periph_AFIO |
                           RCC_APB2Periph_ADC1  | RCC_APB2Periph_TIM1 |
                           RCC_APB2Periph_TIM8, ENABLE);

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5  | RCC_APB1Periph_TIM2 |
                           RCC_APB1Periph_UART4 | RCC_APB1Periph_TIM3 |
                           RCC_APB1Periph_TIM4, ENABLE);

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,  ENABLE);
}

void WatchDog(void)
{
    if (WatchDogCounter++ > 1000)
    {
        LEDtoggle();
        PWMOff();
        DEBUG_PutChar('W');
        WatchDogCounter = 0;
    }
}

static float idlePerf;

float GetIdlePerf(void)
{
    return idlePerf;
}

void setup(void)
{
    InitSysTick();

    Periph_clock_enable();
    GPIO_Config();

    __enable_irq();

    ComInit();
    print("\r\n\r\nEvvGC firmware starting up...\r\n");

    print("init motor PWM...\r\n");
    PWMConfig();

    for (int i = 0; i < 20; i++)
    {
        LEDtoggle();
        DEBUG_LEDtoggle();
        Delay_ms(100); //short blink
    }

    if (GetVCPConnectMode() != eVCPConnectReset)
    {
        print("\r\nUSB startup delay...\r\n");
        Delay_ms(3000);

        if (GetVCPConnectMode() == eVCPConnectData)
        {
            print("\r\n\r\nEvvGC firmware starting up, USB connected...\r\n");
        }
    }
    else
    {
        print("\r\nDelaying for usb/serial driver to settle\r\n");
        Delay_ms(3000);
        print("\r\n\r\nEvvGC firmware starting up, serial active...\r\n");
    }

#ifdef __VERSION__
    print("gcc version " __VERSION__ "\r\n");
    print("EvvGC firmware V%s, build date " __DATE__ " "__TIME__" \r\n", __EV_VERSION);
#endif

    if ((RCC->CR & RCC_CR_HSERDY) != RESET)
    {
        print("running on external HSE clock, clock rate is %dMHz\r\n", SystemCoreClock / 1000000);
    }
    else
    {
        print("ERROR: running on internal HSI clock, clock rate is %dMHz\r\n", SystemCoreClock / 1000000);
    }

    print("init ADC...\r\n");
    ADC_Config();

    print("init MPU6050...\r\n");

    while (MPU6050_Init())
    {
        print("init MPU6050 failed - press 'b' to enter bootloader, retrying...\r\n");
        int c = GetChar();
        if(c=='b'){
        	print("rebooting into boot loader ...\r\n");
            Delay_ms(1000);
            bootloader();
        }
        Blink();
    }

    print("loading config...\r\n");
    configLoad();

    print("calibrating MPU6050 at %ums...\r\n", millis());
    MPU6050_Gyro_calibration();

    print("init RC...\r\n");
    RC_Config();

    print("Init Orientation\n\r");
    Init_Orientation();

    InitSinArray();

    int pendingCharacters = ComFlushInput();

    if (pendingCharacters > 0)
    {
        print("removed %d pending characters from communications input\r\n");
    }

#if 0
    int c;

    while ((c = GetChar()) >= 0)
    {
        print("removed pending character %02X from communications input\r\n", c);
    }

#endif

    print("entering main loop...\r\n");

    SysTickAttachCallback(WatchDog);
}

static int GetIdleMax(void)
{
    unsigned int t0 = millis();

    while (millis() == t0)
        ;

    unsigned int lastTime = micros();
    int idleLoops = 0;

    __disable_irq();

    while (1)
    {
        idleLoops++;
        unsigned int currentTime = micros();
        unsigned int timePassed = currentTime - lastTime;

        if (timePassed >= 500U)
        {
            break;
        }
    }

    __enable_irq();
    int idleMax = 2 * idleLoops; // loops/ms
    idleLoops = 0;

    return idleMax;
}

int main(void)
{
    setup();
    int idleMax = GetIdleMax();

    int idleLoops = 0;
    unsigned int lastTime = micros();

    print("Press '?' for CLI documentation\r\n");
    while (1)
    {
        idleLoops++;
        unsigned int currentTime = micros();
        unsigned int timePassed = currentTime - lastTime;

        if (timePassed >= 2000)
        {
            idlePerf = idleLoops * 100.0 * 1000 / timePassed / idleMax; // perf in percent
            idleLoops = 0;

            if (ConfigMode == 0)
            {
                engineProcess(timePassed / 1000000.0);
            }
            else
            {
                PWMOff();
                Blink();
            }

            WatchDogCounter = 0;
            CommHandler();
            lastTime = currentTime;
        }
    }
}
