/*
 * 	comio.h
 *
 *	Created on: Aug 1, 2013
 *		Author: ala42
 */

#ifndef COMIO_H_
#define COMIO_H_
#include <stdint.h>
#include <stdarg.h>

void DEBUG_PutChar(char ch);
void DEBUG_PutString(char *str);

void print(const char * fmt, ...);
void printDirect(const char * fmt, ...);

int  GetChar(void);
void UnGetChar(unsigned char c);
int  CharAvailable(void);
void PutChar(unsigned char c);
#endif /* COMIO_H_ */
