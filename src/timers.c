/*
 * 	timers.c
 *
 *	Created on: Jun 25, 2013
 *		Author: Denis aka caat
 */

#include "timers.h"
#include "stm32f10x_tim.h"
#include "pins.h"
#include "utils.h"

//TIM_OCInitTypeDef       	TIM_OCInitStructure;
//TIM_TimeBaseInitTypeDef     TIM_TimeBaseStructure;
//TIM_TimeBaseInitTypeDef 	TIM_TimeBaseInitStructure;
//TIM_BDTRInitTypeDef 		TIM_BDTRInitStructure;

void Timer3_Config(void)//RC control timer config
{
    TIM_TimeBaseInitTypeDef 	TIM_TimeBaseInitStructure;
    //Timer3 Config

    TIM_TimeBaseInitStructure.TIM_Prescaler = 719; // Period*Prescaler=24'000'000Hz  //2400 1s
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period = 60000;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure);

    TIM_Cmd(TIM3, ENABLE);
}

void Timer1_Config(void)//Pitch Timer configuration
{
    TIM_TimeBaseInitTypeDef     TIM_TimeBaseInitStructure;
    TIM_OCInitTypeDef       	TIM_OCInitStructure;
    TIM_BDTRInitTypeDef 		TIM_BDTRInitStructure;
    //Timer1 Config

    //Time Base configuration
    TIM_TimeBaseInitStructure.TIM_Prescaler = 3; // Period*Prescaler=24'000'000Hz  //2400 1s
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period = 1000; //20000(presc=24)=50hz(servo signal)
    TIM_TimeBaseInitStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStructure);

    //Configuration in PWM mode
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1 ;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;
    TIM_OC1Init(TIM1, &TIM_OCInitStructure);
    TIM_OC2Init(TIM1, &TIM_OCInitStructure);
    TIM_OC3Init(TIM1, &TIM_OCInitStructure);

    //Automatic Output enable, Break, dead time and lock configuration
    TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
    TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
    TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_1;
    TIM_BDTRInitStructure.TIM_DeadTime = 200;
    TIM_BDTRInitStructure.TIM_Break = TIM_Break_Disable;
    TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_High;
    TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;
    TIM_BDTRConfig(TIM1, &TIM_BDTRInitStructure);

    //TIM1 counter enable
    TIM_Cmd(TIM1, ENABLE);

    //Main Output Enable
    TIM_CtrlPWMOutputs(TIM1, ENABLE);
}

void Timer8_Config(void)//Roll Timer configuration
{
    TIM_TimeBaseInitTypeDef     TIM_TimeBaseInitStructure;
    TIM_OCInitTypeDef       	TIM_OCInitStructure;
    TIM_BDTRInitTypeDef 		TIM_BDTRInitStructure;
    //Timer8 Config

    //Time Base configuration
    TIM_TimeBaseInitStructure.TIM_Prescaler = 3; // Period*Prescaler=24'000'000Hz  //2400 1s
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period = 1000; //20000(presc=24)=50hz(servo signal)
    TIM_TimeBaseInitStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM8, &TIM_TimeBaseInitStructure);

    //Configuration in PWM mode
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1 ;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;
    TIM_OC1Init(TIM8, &TIM_OCInitStructure);
    TIM_OC2Init(TIM8, &TIM_OCInitStructure);
    TIM_OC3Init(TIM8, &TIM_OCInitStructure);


    //Automatic Output enable, Break, dead time and lock configuration
    TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
    TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
    TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_1;
    TIM_BDTRInitStructure.TIM_DeadTime = 200;
    TIM_BDTRInitStructure.TIM_Break = TIM_Break_Disable;
    TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_High;
    TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;
    TIM_BDTRConfig(TIM8, &TIM_BDTRInitStructure);

    //TIM8 counter enable
    TIM_Cmd(TIM8, ENABLE);

    //Main Output Enable
    TIM_CtrlPWMOutputs(TIM8, ENABLE);
}

void Timer2_Config(void)//Main(loop) Timer configuration
{
    //Timer2 Config

    //	TIM2->PSC = 71;         // Set prescaler (PSC + 1)
    TIM2->PSC = TIM2_PRESCALER;	// Set prescaler (PSC + 1)
    //	TIM2->ARR = 2000;           // Auto reload value 2000
    TIM2->ARR = TIM2_AUTORELOADVALUE;           // Auto reload value 2000
    TIM2->DIER = TIM_DIER_UIE; // Enable update interrupt (timer level)
    TIM2->CR1 = TIM_CR1_CEN;   // Enable timer
    NVIC_EnableIRQ(TIM2_IRQn); // Enable interrupt from TIM2 (NVIC level)
}

void Timer5_Config(void)//Pitch Timer configuration
{
    TIM_TimeBaseInitTypeDef     TIM_TimeBaseInitStructure;
    TIM_OCInitTypeDef       	TIM_OCInitStructure;
    //Timer5 Config

    TIM_TimeBaseInitStructure.TIM_Prescaler = 3; // Period*Prescaler=24'000'000Hz  //2400 1s
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period = 1000; //20000(presc=24)=50hz(servo signal)
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV4;
    TIM_TimeBaseInit(TIM5, &TIM_TimeBaseInitStructure);

    //PWM Config on Timer5 CH3
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1 ;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
    TIM_OC3Init(TIM5, &TIM_OCInitStructure);
    //PWM Config on Timer5 CH2
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1 ;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
    TIM_OC2Init(TIM5, &TIM_OCInitStructure);
    //PWM Config on Timer5 CH1
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1 ;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
    TIM_OC1Init(TIM5, &TIM_OCInitStructure);
}

void Timer4_Config(void)//Pitch Timer configuration
{
    TIM_TimeBaseInitTypeDef     TIM_TimeBaseInitStructure;
    TIM_OCInitTypeDef       	TIM_OCInitStructure;
    //Timer4 Config

    TIM_TimeBaseInitStructure.TIM_Prescaler = 3; // Period*Prescaler=24'000'000Hz  //2400 1s
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period = 1000; //20000(presc=24)=50hz(servo signal)
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV4;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStructure);

    //PWM Config on Timer4 CH3
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1 ;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC3Init(TIM4, &TIM_OCInitStructure);
    //PWM Config on Timer4 CH2
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1 ;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC2Init(TIM4, &TIM_OCInitStructure);
    //PWM Config on Timer4 CH1
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1 ;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC1Init(TIM4, &TIM_OCInitStructure);
}

void TimerOff(void)
{
    LEDon;					//blinking led
    Delay_ms(100);
    LEDoff;
    Delay_ms(100);
    TIM4->CCR1 = 0;  	//turns off all motors outputs
    TIM4->CCR2 = 0;
    TIM4->CCR3 = 0;
    TIM5->CCR1 = 1000;
    TIM5->CCR2 = 1000;
    TIM5->CCR3 = 1000;
    TIM1->CCR1 = 0;
    TIM1->CCR2 = 0;
    TIM1->CCR3 = 0;
    TIM8->CCR1 = 0;
    TIM8->CCR2 = 0;
    TIM8->CCR3 = 0;
}

