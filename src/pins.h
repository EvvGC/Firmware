/*
 * 	pins.h
 *
 *	Created on: Jun 25, 2013
 *		Author: Denis caat
 */

#ifndef PINS_H_
#define PINS_H_

#include "stm32f10x_gpio.h"

#define LEDon  GPIO_WriteBit(GPIOB, GPIO_Pin_12,   Bit_SET) //LED on
#define LEDoff GPIO_WriteBit(GPIOB, GPIO_Pin_12, Bit_RESET) //LED off

#define DEBUG_LEDon  GPIO_WriteBit(GPIOB, GPIO_Pin_5,   Bit_SET) //LED on
#define DEBUG_LEDoff GPIO_WriteBit(GPIOB, GPIO_Pin_5, Bit_RESET) //LED off

void GPIO_Config(void);
void Blink(void);

#endif /* PINS_H_ */
