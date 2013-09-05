/*
 *  usart.c
 *
 *  Created on: Jun 26, 2013
 *      Author: Denis aka caat
 */
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "usart.h"
#include "utils.h"
#include "ringbuffer.h"

static tRingBuffer RingBufferUART4TX;
static tRingBuffer RingBufferUART4RX;
unsigned int IrqCntUart4;

void InitUart4Buffer(void);

void InitUart4BufferIRQ(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Configure the NVIC Preemption Priority Bits */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    /* Enable the USARTy Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; //Preemption Priority
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void Usart4Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    USART_ClockInitTypeDef USART_ClockInitStructure;

    //Set USART4 Tx (PC10) as AF push-pull
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    //Set USART4 Rx (PC11) as input pull down
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_Init(GPIOC, &GPIO_InitStructure);


    USART_ClockStructInit(&USART_ClockInitStructure);
    USART_ClockInit(UART4, &USART_ClockInitStructure);
    USART_InitStructure.USART_BaudRate = 9600;
    //USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No ;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(UART4, &USART_InitStructure);

    //Enable UART4 Receive interrupt
    USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);
    //Enable USART4
    USART_Cmd(UART4, ENABLE);

    InitUart4Buffer();
    InitUart4BufferIRQ();
}

int USART_GetChar(void)
{
    return RingBufferGet(&RingBufferUART4RX);
}

int USART_Peek(void)
{
    return RingBufferPeek(&RingBufferUART4RX);
}

int USART_Available(void)
{
    return RingBufferFillLevel(&RingBufferUART4RX);
}

void USART_Flush(void)
{
    while (RingBufferFillLevel(&RingBufferUART4TX) != 0)
        ;
}

void USART_PutCharDirect(uint8_t ch)
{
    while (!(UART4->SR & USART_SR_TXE));

    UART4->DR = ch;
}

void USART_PutChar(uint8_t ch)
{
    //while (!(UART4->SR & USART_SR_TXE));
    //  UART4->DR = ch;
    RingBufferPut(&RingBufferUART4TX, ch, 1);
}

void USART_PutStringDirect(uint8_t *str)
{
    while (*str != 0)
    {
        USART_PutCharDirect(*str);
        str++;
    }
}

void USART_PutString(uint8_t *str)
{
    while (*str != 0)
    {
        USART_PutChar(*str);
        str++;
    }
}

void UART4EnableTxInterrupt(void)
{
    USART_ITConfig(UART4, USART_IT_TXE, ENABLE);
}

void UART4_IRQHandler(void) //UART4 Interrupt handler implementation
{
    int sr = UART4->SR;
    IrqCntUart4++;

    if (sr & USART_FLAG_TXE)
    {
        tRingBuffer *rb = &RingBufferUART4TX;

        if (rb->Read != rb->Write)
        {
            UART4->DR = rb->Buffer[rb->Read];

            if (rb->Read + 1 == RingBufferSize(rb))
            {
                rb->Read = 0;
            }
            else
            {
                rb->Read++;
            }
        }
        else
        {
            USART_ITConfig(UART4, USART_IT_TXE, DISABLE);
            asm volatile("nop");
            asm volatile("nop");
        }
    }

    if (sr & USART_FLAG_RXNE)
    {
        tRingBuffer *rb = &RingBufferUART4RX;

        unsigned char c = UART4->DR;

        if (RingBufferFillLevel(rb) + 1 == RingBufferSize(rb))
        {
            rb->Overrun++;
            return;
        }

        rb->Buffer[rb->Write] = c;

        if (rb->Write + 1 == RingBufferSize(rb))
        {
            rb->Write = 0;
        }
        else
        {
            rb->Write++;
        }
    }
}

void InitUart4Buffer(void)
{
    RingBufferInit(&RingBufferUART4TX, &UART4EnableTxInterrupt);
    RingBufferInit(&RingBufferUART4RX, 0L);
}
