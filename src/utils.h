/*
 *  utils.h
 *
 *  Created on: Jun 25, 2013
 *      Author: Denis caat
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

#ifndef UTILS_H_
#define UTILS_H_

void LEDon(void);
void LEDoff(void);
void LEDtoggle(void);

void DEBUG_LEDon(void);
void DEBUG_LEDoff(void);
void DEBUG_LEDtoggle(void);

void Blink(void);

void Delay_ms(unsigned int ms);
void Delay_us(unsigned int us);

float Rad2Deg(float x);
float Deg2Rad(float x);
float Round(float x);

#define ARRAY_SIZE(x)			(sizeof(x)/sizeof(*(x)))

#endif /* UTILS_H_ */
