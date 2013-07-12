/*
 * 	eeprom.c
 *
 *	Created on: Jun 25, 2013
 *		Author: Denis aka caat
 */

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

uint8_t ReadFromEEPROM(uint8_t readAddr)
{
    uint8_t data;
    Delay_ms(1);

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
    return data;
}

