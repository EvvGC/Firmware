/*
#include "config.h"
#include "pins.h"
#include "usart.h"
#include "utils.h"
#include "adc.h"
#include "timers.h"
#include "eeprom.h"
#include "engine.h"
#include "gyro.h"
#include "pwm.h"
#include "systick.h"
*/

#include "rc.h"
#include "stm32f10x_tim.h"
#include "utils.h"
#include "comio.h"


void Timer3_Config(void) // RC control timer config
{
    TIM_TimeBaseInitTypeDef 	TIM_TimeBaseInitStructure;

    TIM_TimeBaseInitStructure.TIM_Prescaler = 71; // 1MHz
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period = 0xffff;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure);

    TIM_Cmd(TIM3, ENABLE);
}


void RC_Config(void)
{
    GPIO_InitTypeDef	GPIO_InitStructure;

	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable , ENABLE);
	
    //EXTI IN GPIO Config
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4; //PB3-Pitch   PB4-Yaw
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;  //Set to Inpit
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;      //GPIO Speed
    GPIO_Init(GPIOB, &GPIO_InitStructure);


    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource3);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource4);

	EXTI_InitTypeDef EXTI_InitStructure;

    EXTI_InitStructure.EXTI_Line = EXTI_Line3 | EXTI_Line4;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    EXTI_GenerateSWInterrupt(EXTI_Line3 | EXTI_Line4);
    EXTI_ClearITPendingBit(EXTI_Line3 | EXTI_Line4);

    NVIC_EnableIRQ(EXTI3_IRQn); // Enable interrupt
    NVIC_EnableIRQ(EXTI4_IRQn); // Enable interrupt
	
    Timer3_Config();//RC control timer
}


static int rc3;
int GetAUX3(void)
{
	return rc3;
}


void EXTI3_IRQHandler(void)//EXTernal interrupt routine PB3-Pitch
{
	static unsigned short rc3a, rc3b;
    if (EXTI->PR & (1 << 3))
    {
		DEBUG_PutChar('3');
        // EXTI3 interrupt pending?
        EXTI->PR |= (1 << 3); // clear pending interrupt

        if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_3) == 1)
        {
            rc3a = TIM3->CNT;
        }

        if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_3) == 0)
        {
            rc3b = TIM3->CNT;
        }

		unsigned short diff = rc3b - rc3a;
        if ((diff > 1000) && (diff < 2000))
        {
            rc3 = (int)diff - 1000;
        }
    }
}

static int rc4;
int GetAUX4(void)
{
	return rc4;
}
	
void EXTI4_IRQHandler(void)//EXTernal interrupt routine PB4-Yaw
{
	static unsigned short rc4a, rc4b;

    if (EXTI->PR & (1 << 4))
    {
		DEBUG_PutChar('4');

        // EXTI3 interrupt pending?
        EXTI->PR |= (1 << 4);                         // clear pending interrupt


        if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_4) == 1)
        {
            rc4a = TIM3->CNT;
        }

        if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_4) == 0)
        {
            rc4b = TIM3->CNT;
        }

		unsigned short diff = rc4b - rc4a;
        if ((diff > 1000) && (diff < 2000))
        {
            rc4 = (int)diff - 1000;
        }
    }
}

