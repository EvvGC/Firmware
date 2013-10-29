/*
 *  i2c.h
 *
 *  Created on: Jun 25, 2013
 *      Author: Denis aka caat
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

#ifndef I2C_H_
#define I2C_H_

//extern int I2Cerror = 0;      //need them to make local!;
//extern int I2Cerrorcount = 0; //need to make them local;
extern int I2Cerror;        //need them to make local!;
extern int I2Cerrorcount;   //need to make them local;

#define I2C_SDA_PIN     GPIO_Pin_11
#define I2C_SDA_PORT    GPIOB

#define I2C_SCL_PIN     GPIO_Pin_10
#define I2C_SCL_PORT    GPIOB

#define SDAH()  GPIO_WriteBit(I2C_SDA_PORT, I2C_SDA_PIN,   Bit_SET)
#define SDAL()  GPIO_WriteBit(I2C_SDA_PORT, I2C_SDA_PIN,   Bit_RESET)

#define SCLH()  GPIO_WriteBit(I2C_SCL_PORT, I2C_SCL_PIN,   Bit_SET)
#define SCLL()  GPIO_WriteBit(I2C_SCL_PORT, I2C_SCL_PIN,   Bit_RESET)

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
