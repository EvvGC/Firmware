/*
 *  usart.h
 *
 *  Created on: Jun 26, 2013
 *      Author: Denis aka caat
 */

#ifndef USART_H_
#define USART_H_

extern unsigned int IrqCntUart4;

void Usart4Init(void);
void USART_PutChar(uint8_t ch);
void USART_PutString(uint8_t *str);
void USART_PutCharDirect(uint8_t ch);
void USART_PutStringDirect(uint8_t *str);
void USART_Flush(void);

int USART_GetChar(void);
int USART_PeekChar(void);
int USART_Available(void);

void InitUart4Buffer(void);
void TestUart4Buffer(void);
void UART4_IRQHandler(void);

#endif /* USART_H_ */
