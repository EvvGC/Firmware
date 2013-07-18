/*
 * 	usart.h
 *
 *	Created on: Jun 26, 2013
 *		Author: Denis aka caat
 */

#ifndef USART_H_
#define USART_H_

#include <stdint.h>

void Usart4Init(void);
void USART_PutChar(uint8_t ch);
void USART_PutString(uint8_t * str);

#endif /* USART_H_ */
