/*
 *  pwm.c
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

#include <math.h>
#include "pwm.h"
#include "stm32f10x.h"
#include "utils.h"
#include "comio.h"
#include "stopwatch.h"
#include "fasttrig.h"
#include "definitions.h"

/*
    PWM timer usage:

    TIM8 Roll
        PC6, PC7, PC8 used for Roll,  TIM_OCPolarity_High
        PA7, PB0, PB1 used for RollN, TIM_OCPolarity_High

    TIM1 Pitch
        PA8,  PA9,  PA10 used for Pitch, TIM_OCPolarity_High
        PB13, PB14, PB15 used for PitchN, TIM_OCPolarity_High

    TIM5 Yaw
        PA0, PA1, PA2 used for Yaw, TIM_OCPolarity_High

    TIM4 YawN
        PB6, PB7, PB8 used for YawN, TIM_OCPolarity_Low

*/


int timer_1_8_deadtime_register = 200; //this is not just a delay value, check CPU reference manual for TIMx_BDTR DTG bit 0-7
int timer_4_5_deadtime_delay = 80; // in 18MHz ticks

float testPhase = -0.09;

static int g_YawOff = 1;
static int g_Roll[3], g_Pitch[3], g_Yaw[3];

int MaxCnt[NUMAXIS];
int MinCnt[NUMAXIS];
int IrqCnt[NUMAXIS];

void MaxCntClear(void)
{
    IrqCnt[ROLL] = IrqCnt[PITCH] = IrqCnt[YAW] = 0;
    MaxCnt[ROLL] = MaxCnt[PITCH] = MaxCnt[YAW] = 0;
    MinCnt[ROLL] = MinCnt[PITCH] = MinCnt[YAW] = PWM_PERIODE + 1;
}

void SetPWMData(int *target, int *pwm)
{
    __disable_irq();

    target[0] = pwm[0];
    target[1] = pwm[1];
    target[2] = pwm[2];

    __enable_irq();
}

void LimitYawPWM(int *pwm)
{
    int maxVal = PWM_PERIODE - 2 * timer_4_5_deadtime_delay;

    for (int i = 0; i < 3; i++)
    {
        pwm[i] -= timer_4_5_deadtime_delay;

        if (pwm[i] >= maxVal)
        {
            pwm[i] = maxVal;
        }

        if (pwm[i] < timer_4_5_deadtime_delay)
        {
            pwm[i] = 0;
        }
    }
}

int dist(int a, int b)
{
    int d = a - b;

    if (d < 0) d = -d;

    return d > 3;
}

void SetPWMOrg(int *pwm, float output, int level)
{
    if (testPhase >= 0)
    {
        output = testPhase;
    }

    pwm[0] = (sin(output)        * 5 * level) + (PWM_PERIODE / 2);
    pwm[1] = (sin(output + 2.09) * 5 * level) + (PWM_PERIODE / 2);
    pwm[2] = (sin(output + 4.19) * 5 * level) + (PWM_PERIODE / 2);
}

void SetPWMOrgFaster(int *pwm, float phi, int power)
{
    if (testPhase >= 0)
    {
        phi = testPhase;
    }

    float fPower = 5 * power;

    phi = fmodf(phi, M_TWOPI); // sinf gets slow if phi >201
    pwm[0] = (int)(sinf(phi)                * fPower + 0.5F) + (PWM_PERIODE / 2);
    pwm[1] = (int)(sinf(phi + 2.0 / 3.0 * M_PI) * fPower + 0.5F) + (PWM_PERIODE / 2);
    pwm[2] = (int)(sinf(phi + 4.0 / 3.0 * M_PI) * fPower + 0.5F) + (PWM_PERIODE / 2);
}

void SetPWMFastTable(int *pwm, float phi, int power)
{
    if (testPhase >= 0)
    {
        phi = testPhase;
    }

    int phiInt = (int)Round(phi / M_TWOPI * SINARRAYSIZE);
    phiInt = phiInt % SINARRAYSIZE;

    if (phiInt < 0)
    {
        phiInt = SINARRAYSIZE + phiInt;
    }

    int iPower = 5 * power;
    pwm[0] = (sinDataI16[phiInt                          % SINARRAYSIZE] * iPower + SINARRAYSCALE / 2) / SINARRAYSCALE + (PWM_PERIODE / 2);
    pwm[1] = (sinDataI16[(phiInt + 1 * SINARRAYSIZE / 3)     % SINARRAYSIZE] * iPower + SINARRAYSCALE / 2) / SINARRAYSCALE + (PWM_PERIODE / 2);
    pwm[2] = (sinDataI16[(phiInt + (2 * SINARRAYSIZE + 1) / 3) % SINARRAYSIZE] * iPower + SINARRAYSCALE / 2) / SINARRAYSCALE + (PWM_PERIODE / 2);
}

void SetPWM(int *pwm, float phi, int power)
{
    //SetPWMOrg(pwm, phi, power);
    SetPWMFastTable(pwm, phi, power);
}

void ActivateIRQ(TIM_TypeDef *tim)
{
    __disable_irq();
    tim->SR &= ~TIM_SR_UIF;   // clear UIF flag
    tim->DIER = TIM_DIER_UIE; // Enable update interrupt
    __enable_irq();
}

void SetRollMotor(float phi, int power)
{
    int pwm[3];
    SetPWM(pwm, phi, power);
    SetPWMData(g_Roll, pwm);

#if 0
    static float lastTestPhase;

    if (testPhase != lastTestPhase)
    {
        print("testPhase %f lastTestPhase %f, roll %d %d %d\r\n", testPhase, lastTestPhase, g_Roll[0], g_Roll[1], g_Roll[2]);
    }

    lastTestPhase = testPhase;
#endif

    ActivateIRQ(TIM8);
}

void SetPitchMotor(float phi, int power)
{
    int pwm[3];
    SetPWM(pwm, phi, power);
    SetPWMData(g_Pitch, pwm);
    ActivateIRQ(TIM1);
}

void SetYawMotor(float phi, int power)
{
    int pwm[3];
    SetPWM(pwm, phi, power);
    LimitYawPWM(pwm);
    SetPWMData(g_Yaw, pwm);
    g_YawOff = 0;
    ActivateIRQ(TIM5);
}

inline void UpdateCounter(tAxis channel, int value)
{
    IrqCnt[channel]++;

    if (value > MaxCnt[channel])
    {
        MaxCnt[channel] = value;
    }

    if (value < MinCnt[channel])
    {
        MinCnt[channel] = value;
    }
}

#define MAX_CNT (PWM_PERIODE * 8 / 10)

void TIM5_IRQHandler(void) // yaw axis
{
    if (TIM5->SR & TIM_SR_UIF) // if UIF flag is set
    {
        TIM5->SR &= ~TIM_SR_UIF; // clear UIF flag

        __disable_irq();
        unsigned short cnt = TIM5->CNT;
        UpdateCounter(YAW, cnt);

        if (cnt < MAX_CNT)  // make sure there is enough time to make all changes
        {
            if (g_YawOff)
            {
                TIM4->CCR1 = PWM_PERIODE + 1;
                TIM4->CCR2 = PWM_PERIODE + 1;
                TIM4->CCR3 = PWM_PERIODE + 1;

                TIM5->CCR1 = 0;
                TIM5->CCR2 = 0;
                TIM5->CCR3 = 0;
            }
            else
            {
                int deadTime = 2 * timer_4_5_deadtime_delay;
                TIM4->CCR1 = g_Yaw[0] + deadTime;
                TIM4->CCR2 = g_Yaw[1] + deadTime;
                TIM4->CCR3 = g_Yaw[2] + deadTime;

                TIM5->CCR1 = g_Yaw[0];
                TIM5->CCR2 = g_Yaw[1];
                TIM5->CCR3 = g_Yaw[2];
            }

            TIM5->DIER &= ~TIM_DIER_UIE;  // disable update interrupt
        }

        __enable_irq();
    }
}

void TIM1_UP_IRQHandler(void) // pitch axis
{
    TIM1->SR &= ~TIM_SR_UIF; // clear UIF flag

    __disable_irq();
    unsigned short cnt = TIM1->CNT;
    UpdateCounter(PITCH, cnt);

    if (cnt < MAX_CNT)  // make sure there is enough time to make all changes
    {
        TIM1->CCR1 = g_Pitch[0];
        TIM1->CCR2 = g_Pitch[1];
        TIM1->CCR3 = g_Pitch[2];

        TIM1->DIER &= ~TIM_DIER_UIE; // disable update interrupt
    }

    __enable_irq();
}

void TIM8_UP_IRQHandler(void) // roll axis
{
    TIM8->SR &= ~TIM_SR_UIF; // clear UIF flag

    __disable_irq();
    unsigned short cnt = TIM8->CNT;
    UpdateCounter(ROLL, cnt);

    if (cnt < MAX_CNT)  // make sure there is enough time to make all changes
    {
        TIM8->CCR1 = g_Roll[0];
        TIM8->CCR2 = g_Roll[1];
        TIM8->CCR3 = g_Roll[2];

        TIM8->DIER &= ~TIM_DIER_UIE; // disable update interrupt
    }

    __enable_irq();
}

static void Timer_Channel_Config(TIM_TypeDef *tim, TIM_OCInitTypeDef *OCInitStructure)
{
    TIM_OC1Init(tim, OCInitStructure);
    TIM_OC2Init(tim, OCInitStructure);
    TIM_OC3Init(tim, OCInitStructure);

    TIM_OC1PreloadConfig(tim, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(tim, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(tim, TIM_OCPreload_Enable);
}

static void Timer_PWM_Advanced_Config(TIM_TypeDef *tim)
{
    TIM_TimeBaseInitTypeDef     TIM_TimeBaseInitStructure;
    TIM_OCInitTypeDef           TIM_OCInitStructure;
    TIM_BDTRInitTypeDef         TIM_BDTRInitStructure;

    //Time Base configuration
    TIM_TimeBaseInitStructure.TIM_Prescaler = 3; // 18MHz
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period = PWM_PERIODE;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(tim, &TIM_TimeBaseInitStructure);

    //Automatic Output enable, Break, dead time and lock configuration
    TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
    TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
    TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_1;
    TIM_BDTRInitStructure.TIM_DeadTime = timer_1_8_deadtime_register;
    TIM_BDTRInitStructure.TIM_Break = TIM_Break_Disable;
    TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_High;
    TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;
    TIM_BDTRConfig(tim, &TIM_BDTRInitStructure);

    //Configuration in PWM mode
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1 ;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;

    Timer_Channel_Config(tim, &TIM_OCInitStructure);
}

static void Timer_PWM_General_Config(TIM_TypeDef *tim, int polarity)
{
    TIM_TimeBaseInitTypeDef     TIM_TimeBaseInitStructure;
    TIM_OCInitTypeDef           TIM_OCInitStructure;

    TIM_TimeBaseInitStructure.TIM_Prescaler = 3; // 18MHz
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period = PWM_PERIODE;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV4;
    TIM_TimeBaseInit(tim, &TIM_TimeBaseInitStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1 ;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OCInitStructure.TIM_OCPolarity = polarity;

    Timer_Channel_Config(tim, &TIM_OCInitStructure);
}

void PWMOff(void)
{
    DEBUG_PutChar('A');

    int pwm[3];
    pwm[0] = pwm[1] = pwm[2] = 0;
    SetPWMData(g_Roll, pwm);
    SetPWMData(g_Pitch, pwm);

    g_YawOff = 1;
    TIM5->DIER = TIM_DIER_UIE; // Enable update interrupt
    TIM1->DIER = TIM_DIER_UIE; // Enable update interrupt
    TIM8->DIER = TIM_DIER_UIE; // Enable update interrupt
}

static void SetupPWMIrq(uint8_t irq)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_InitStructure.NVIC_IRQChannel = irq;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;//Preemption Priority
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

#define BB_PERIPH_ADDR(addr, bit) ((vu32*)(PERIPH_BB_BASE + ((void*)(addr)-(void*)PERIPH_BASE) * 32 + (bit) * 4))

void PWMConfig(void)
{
    MaxCntClear();

    //rewrite that thing;
    Timer_PWM_Advanced_Config(TIM1);
    Timer_PWM_Advanced_Config(TIM8);

    Timer_PWM_General_Config(TIM5, TIM_OCPolarity_High);
    Timer_PWM_General_Config(TIM4, TIM_OCPolarity_Low);

    TIM4->CNT = timer_4_5_deadtime_delay;
    TIM1->CNT = timer_4_5_deadtime_delay + 3 + PWM_PERIODE / 3;
    TIM8->CNT = timer_4_5_deadtime_delay + 5 + PWM_PERIODE * 2 / 3;

    SetupPWMIrq(TIM5_IRQn);    // yaw
    SetupPWMIrq(TIM1_UP_IRQn); // pitch
    SetupPWMIrq(TIM8_UP_IRQn); // roll

    __disable_irq();
    {
        /* code below is faster version of
        TIM_Cmd(TIM5, ENABLE);
        TIM_Cmd(TIM4, ENABLE);
        */
        vu32 *tim5Enable = BB_PERIPH_ADDR(&(TIM5->CR1), 0);
        vu32 *tim4Enable = BB_PERIPH_ADDR(&(TIM4->CR1), 0);
        vu32 *tim1Enable = BB_PERIPH_ADDR(&(TIM1->CR1), 0);
        vu32 *tim8Enable = BB_PERIPH_ADDR(&(TIM8->CR1), 0);
        *tim5Enable = 1;
        *tim4Enable = 1;
        *tim1Enable = 1;
        *tim8Enable = 1;
    }

    TIM_CtrlPWMOutputs(TIM5, ENABLE);
    TIM_CtrlPWMOutputs(TIM4, ENABLE);
    TIM_CtrlPWMOutputs(TIM1, ENABLE);
    TIM_CtrlPWMOutputs(TIM8, ENABLE);
    __enable_irq();
}
