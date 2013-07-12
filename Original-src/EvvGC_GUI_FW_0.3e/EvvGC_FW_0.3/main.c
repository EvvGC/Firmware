#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include <stdio.h>

#include "config.h"
#include "pins.h"
#include "utils.h"
#include "adc.h"
#include "timers.h"
#include "eeprom.h"
#include "i2c.h"
#include "engine.h"
#include "gyro.h"

EXTI_InitTypeDef        	EXTI_InitStructure;

void Periph_clock_enable(void); //Enabling clocks for peripheral
void NVIC_Configuration(void);
void UART4_IRQHandler();
void EXTI_Config(void);

int stop=0, EepromData, UART4_DATA, rc3a, rc3b, rc3, rc4a, rc4b, rc4, ConfigMode, w, enable_writing, watchcounter, I2Cerror, I2Cerrorcount;
short int gyroADC_PITCH, gyroADC_ROLL, gyroADC_YAW, accADC_ROLL, accADC_PITCH, accADC_YAW;
char buff[10], configData[configDataSize]={'1','1','1','1','1','1','1','1','1','1','1','1'};



int main(void)
{
	Periph_clock_enable(); 
	GPIO_Config();	
	
	LEDon;
	Delay_ms(10); //short blink
	LEDoff;
	Delay_ms(50);	
	
	Usart4Init();
	ADC_Config();	
	MPU6050_Init();
	Timer2_Config();
	Timer3_Config();//RC control timer
	NVIC_Configuration();
	EXTI_Config();
	

	//engineInit();	///????to initialize all variables;
	configLoad();
							
	MPU6050_Gyro_calibration();

	while(1)
	{	
		engineProcess();
		while(stop==0) {}//Closed loop waits for interrupt	
	}
}

void Periph_clock_enable(void)
{
	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
                         	RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD |
                         	RCC_APB2Periph_GPIOE | RCC_APB2Periph_AFIO | 
						 	RCC_APB2Periph_ADC1	| RCC_APB2Periph_TIM1 | 
						 	RCC_APB2Periph_TIM8, ENABLE);
	RCC_APB1PeriphClockCmd(	RCC_APB1Periph_TIM5  | RCC_APB1Periph_TIM2 | 
							RCC_APB1Periph_UART4 | RCC_APB1Periph_TIM3 | 
							RCC_APB1Periph_TIM4, ENABLE);
	RCC_AHBPeriphClockCmd (	RCC_AHBPeriph_DMA1,  ENABLE);
}

void UART4_IRQHandler()//UART4 Interrupt handler implementation
{
	int eeRreg;
	uint8_t data;
	ConfigMode=1;
	while ( USART_GetFlagStatus(UART4, USART_FLAG_RXNE) == RESET);
	UART4_DATA=USART_ReceiveData(UART4);
	LEDon;
	if(UART4_DATA==103)
	{ //if "g"
	
		Delay_ms(100);
		sprintf (buff, "x");
		USART_PutString(buff);
		for(eeRreg=0; eeRreg<configDataSize;eeRreg++)
		{
			data = ReadFromEEPROM(eeRreg);
			Delay_ms(1);
			sprintf (buff, "%c", data);
			USART_PutString(buff);
		}
	
	}
	
	if(enable_writing==1)
	{							
		configData[w]=(int)UART4_DATA;
		w++;
		if(w>=configDataSize)
		{
			w=0; 
			enable_writing=0; 
			//saveData();	
			configSave();
		}
	}
	
	
	if(UART4_DATA==104)
	{ // if h (write to eeprom)
		enable_writing=1;
	}
	
	
	if(UART4_DATA==105)
	{ 
		ConfigMode=1;
	}	
	
	if(UART4_DATA==106)
	{
		ConfigMode=0;
	}								
}

void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Configure the NVIC Preemption Priority Bits */  
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
  
  /* Enable the USARTy Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

}

void EXTI_Config(void)
{
	GPIO_InitTypeDef	GPIO_InitStructure;	
	
	//EXTI IN GPIO Config
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4; //PB3-Pitch   PB4-Yaw
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;  //Set to Inpit
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;      //GPIO Speed
	GPIO_Init(GPIOB, &GPIO_InitStructure); 
	
	
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource3);	
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource4);	
	
		
	EXTI_InitStructure.EXTI_Line = EXTI_Line3 | EXTI_Line4;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
	
	EXTI_GenerateSWInterrupt(EXTI_Line3 | EXTI_Line4);	
	EXTI_ClearITPendingBit(EXTI_Line3 | EXTI_Line4);
		
	NVIC_EnableIRQ(EXTI3_IRQn); // Enable interrupt 
	NVIC_EnableIRQ(EXTI4_IRQn); // Enable interrupt 
}

void EXTI3_IRQHandler(void)//EXTernal interrupt routine PB3-Pitch
{
	if (EXTI->PR & (1<<3)) 
	{                        // EXTI3 interrupt pending?
		EXTI->PR |= (1<<3);  // clear pending interrupt

		if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_3)==1)
		{
			rc3a=TIM3->CNT;
		}
		if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_3)==0)
		{
			rc3b=TIM3->CNT;
		}
		if (((rc3b-rc3a)>100) && ((rc3b-rc3a)<200))
		{
			rc3=rc3b-rc3a-100;
		}
	}
}

void EXTI4_IRQHandler(void)//EXTernal interrupt routine PB4-Yaw
{
	if (EXTI->PR & (1<<4)) 
	{                        // EXTI3 interrupt pending?
		EXTI->PR |= (1<<4);                           // clear pending interrupt
		
			
		if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_4)==1)
		{
			rc4a=TIM3->CNT;
		}
		if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_4)==0)
		{
			rc4b=TIM3->CNT;
		}
		if (((rc4b-rc4a)>100) && ((rc4b-rc4a)<200))
		{
			rc4=rc4b-rc4a-100;
		}
	}
}

void TIM2_IRQHandler(void)
{
	if(TIM2->SR & TIM_SR_UIF) // if UIF flag is set
	{
		TIM2->SR &= ~TIM_SR_UIF; // clear UIF flag  
		stop=1;
		if(ConfigMode==0)
		{
			watchcounter++;
		}
		if(watchcounter > 250)
		{
			TimerOff();
		}	
	}
}
