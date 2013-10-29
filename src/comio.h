/*
 *  comio.h
 *
 *  Created on: Aug 1, 2013
 *      Author: ala42
 */

/*
    Original work Copyright (c) 2012 [Evaldis - RCG user name]
    Modified work Copyright 2012 Alan K. Adamson

    This file is part of EvvGC.

    EvvGC is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    EvvGC is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with EvvGC.  If not, see <http://www.gnu.org/licenses/>.

*/

#ifndef COMIO_H_
#define COMIO_H_

void DEBUG_PutChar(char ch);
void DEBUG_PutString(char *str);

void print(const char *fmt, ...);
void printDirect(const char *fmt, ...);
void printUSART(const char *fmt, ...);

int  GetChar(void);
void UnGetChar(unsigned char c);
int  CharAvailable(void);
void PutChar(unsigned char c);
void ComInit(void);
int ComFlushInput(void);
#endif /* COMIO_H_ */
