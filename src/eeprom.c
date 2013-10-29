/*
 *  eeprom.c
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

#include "stdint.h"
#include "eeprom.h"
#include "utils.h"
#include "i2c.h"

void WriteToEEPROM(uint8_t addressToWrite, uint8_t DataToWrite)//Write data to external EEPROM
{
    Delay_ms(5);

    I2C1_Start();
    I2C1_SendByte((0xAF & 0xFE));//fe-0(Write)
    I2C1_WaitAck();
    I2C1_SendByte(addressToWrite);
    I2C1_WaitAck();
    I2C1_SendByte(DataToWrite);
    I2C1_WaitAck();
    I2C1_Stop();

    Delay_ms(5);
}

int ReadFromEEPROM(uint8_t readAddr)
{
    uint8_t data;
    Delay_ms(1);

    I2Cerror = 0;

    I2C1_Start();
    I2C1_SendByte((0xAF & 0xFE));//fe-0(Write)
    I2C1_WaitAck();
    I2C1_SendByte(readAddr);
    I2C1_WaitAck();
    I2C1_Stop();

    Delay_ms(1);

    I2C1_Start();
    I2C1_SendByte((0xAF & 0xFF));//ff-1(Read)
    I2C1_WaitAck();
    data = I2C1_ReceiveByte();//receive
    I2C1_NoAck();
    I2C1_Stop();

    Delay_ms(1);

    if (I2Cerror != 0)
    {
        return -1;
    }
    else
    {
        return data;
    }
}

