/*
 * 	i2c.h
 *
 *	Created on: Jun 25, 2013
 *		Author: Denis aka caat
 */

#ifndef I2C_H_
#define I2C_H_

#include <stdint.h>
#include "stm32f10x_gpio.h"

//extern int I2Cerror = 0;		//need them to make local!;
//extern int I2Cerrorcount = 0;	//need to make them local;
extern int I2Cerror;		//need them to make local!;
extern int I2Cerrorcount;	//need to make them local;

#define SDAH GPIO_WriteBit(GPIOB, GPIO_Pin_11,   Bit_SET);
#define SCLH GPIO_WriteBit(GPIOB, GPIO_Pin_10,   Bit_SET);

#define SDAL GPIO_WriteBit(GPIOB, GPIO_Pin_11,   Bit_RESET);
#define SCLL GPIO_WriteBit(GPIOB, GPIO_Pin_10,   Bit_RESET);

void I2C_delay(void);
void I2C1_Start(void);
void I2C1_Stop(void);
void I2C1_Ack(void);
void I2C1_NoAck(void);
void I2C1_SendByte(unsigned char SendByte);
void I2C1_WaitAck(void);
//uint8_t I2C1_CheckAck(void);
uint8_t I2C1_ReceiveByte(void);

#endif /* I2C_H_*/
