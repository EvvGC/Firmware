/*
 *  utils.h
 *
 *  Created on: Jun 25, 2013
 *      Author: Denis caat
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
#endif /* UTILS_H_ */
