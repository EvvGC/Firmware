/*
 * 	comio.h
 *
 *	Created on: Aug 1, 2013
 *		Author: ala42
 */

#include "comio.h"
#include <stdio.h>
#include "usart.h"


void print(const char * fmt, ...)
{
	char buf[256];

	va_list vlist;
	va_start (vlist, fmt);

	vsnprintf (buf, sizeof(buf)-1, fmt, vlist);
	USART_PutString((unsigned char*)buf);
	va_end (vlist);
}


void printDirect(const char * fmt, ...)
{
	char buf[256];

	va_list vlist;
	va_start (vlist, fmt);

	vsnprintf (buf, sizeof(buf)-1, fmt, vlist);
	USART_PutStringDirect((unsigned char*)buf);
	va_end (vlist);
}


static int lastChar=-1;
void UnGetChar(unsigned char c)
{
	lastChar = c;
}
	
int GetChar(void)
{
	if(lastChar < 0) {
		return USART_GetChar();
	} else {
		int c = lastChar;
		lastChar = -1;
		return c;
	}
}

int CharAvailable(void)
{
	return USART_Available();
}

void PutChar(unsigned char c)
{
	USART_PutChar(c);
}

void DEBUG_PutChar(char ch)
{
    //USART_PutChar((uint8_t) ch);
}

void DEBUG_PutString(char *str)
{
	USART_PutString((uint8_t *)str);
}

