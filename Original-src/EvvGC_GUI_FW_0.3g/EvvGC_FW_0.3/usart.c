/*
 * 	usart.c
 *
 *	Created on: Jun 26, 2013
 *		Author: Denis aka caat
 */
#include "stm32f10x_gpio.h"
#include "usart.h"

void Usart4Init(void) 
{ 
	GPIO_InitTypeDef GPIO_InitStructure; 
	USART_InitTypeDef USART_InitStructure; 
	
	USART_ClockInitTypeDef USART_ClockInitStructure; 
	
	//Set USART2 Tx (PA.02) as AF push-pull 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_Init(GPIOC, &GPIO_InitStructure); 
	//Set USART2 Rx (PA.03) as input floating 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; 
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	
	USART_ClockStructInit(&USART_ClockInitStructure);
	USART_ClockInit(UART4, &USART_ClockInitStructure); 
	USART_InitStructure.USART_BaudRate = 9600; 
	USART_InitStructure.USART_WordLength = USART_WordLength_8b; 
	USART_InitStructure.USART_StopBits = USART_StopBits_1; 
	USART_InitStructure.USART_Parity = USART_Parity_No ; 
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; 
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; 
	//Write USART2 parameters 
	USART_Init(UART4, &USART_InitStructure); 
	//Enable UART4 Receive interrupt
	USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);
	//Enable USART2 
	USART_Cmd(UART4, ENABLE); 
}

void USART_PutChar(uint8_t ch)
{
  while(!(UART4->SR & USART_SR_TXE));
  UART4->DR = ch;
}

void USART_PutString(uint8_t * str)
{
	while(*str != 0)
	{
		USART_PutChar(*str);
		str++;
	}
}

