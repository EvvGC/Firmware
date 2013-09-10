/*
 *  comio.h
 *
 *  Created on: Aug 1, 2013
 *      Author: ala42
 */
#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>
#include <string.h>
#include "comio.h"
#include "usart.h"
#include "usb.h"

void ComInit(void)
{
    Usart4Init();
    DEBUG_PutString("setup USB...\r\n");
    setupUSB();
    DEBUG_PutString("setup USB done\r\n");
}

int ComFlushInput(void)
{
    int loop;

    for (loop = 0; GetChar() >= 0 && loop < 100; loop++)
        ;

    return loop;
}

void print(const char *fmt, ...)
{
    char buf[256];

    va_list vlist;
    va_start(vlist, fmt);

    vsnprintf(buf, sizeof(buf) - 1, fmt, vlist);
    USART_PutString((unsigned char *)buf);
    usbSendBytes((unsigned char *)buf, strlen(buf));
    va_end(vlist);
}

void printUSART(const char *fmt, ...)
{
    char buf[256];

    va_list vlist;
    va_start(vlist, fmt);

    vsnprintf(buf, sizeof(buf) - 1, fmt, vlist);
    USART_PutString((unsigned char *)buf);
    va_end(vlist);
}

void printDirect(const char *fmt, ...)
{
    char buf[256];

    va_list vlist;
    va_start(vlist, fmt);

    vsnprintf(buf, sizeof(buf) - 1, fmt, vlist);
    USART_PutStringDirect((unsigned char *)buf);
    va_end(vlist);
}

static int lastChar = -1;

void UnGetChar(unsigned char c)
{
    lastChar = c;
}

int GetChar(void)
{
    if (lastChar < 0)
    {
        if (usbBytesAvailable())
        {
            unsigned char c;
            usbReceiveBytes(&c, 1);
            return (c);
        }
        else
        {
            return USART_GetChar();
        }
    }
    else
    {
        int c = lastChar;
        lastChar = -1;
        return c;
    }
}

int CharAvailable(void)
{
    if (usbBytesAvailable())
    {
        return usbBytesAvailable();
    }
    else
    {
        return USART_Available();
    }
}

void PutChar(unsigned char c)
{
    USART_PutChar(c);
    usbSendBytes(&c, 1);
}

void DEBUG_PutChar(char ch)
{
    ch = ch;
    //USART_PutChar((uint8_t) ch);
}

void DEBUG_PutString(char *str)
{
    USART_PutString((uint8_t *)str);
    USART_Flush();
}

