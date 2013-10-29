/*
 *  i2c.c
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

#include <stdint.h>
#include "i2c.h"
#include "pins.h"
#include "utils.h"
#include "stm32f10x.h"

int I2Cerror, I2Cerrorcount;

void I2C_delay(void)
{
    Delay_us(1);
}

void I2C1_Start(void)
{
    SDAH();
    SCLH();
    I2C_delay();
    SDAL();
    I2C_delay();
    SCLL();
    I2C_delay();

}

void I2C1_Stop(void)
{
    SCLL();
    I2C_delay();
    SDAL();
    I2C_delay();
    SCLH();
    I2C_delay();
    SDAH();
    I2C_delay();
}

void I2C1_Ack(void)
{
    SCLL();
    I2C_delay();
    SDAL();
    I2C_delay();
    SCLH();
    I2C_delay();
    SCLL();
    I2C_delay();
}


void I2C1_NoAck(void)
{
    SCLL();
    I2C_delay();
    SDAH();
    I2C_delay();
    SCLH();
    I2C_delay();
    SCLL();
    I2C_delay();
}

void I2C1_SendByte(unsigned char SendByte)
{
    //    int8_t i = 8;
    unsigned char i = 8;

    while (i--)
        //  for(; i > 0; i--)
    {
        SCLL();
        I2C_delay();

        if (SendByte & 0x80)
        {
            SDAH();
        }

        if (!(SendByte & 0x80))
        {
            SDAL();
        }

        SendByte <<= 1;
        I2C_delay();
        SCLH();
        I2C_delay();
    }

    SCLL();
}

uint8_t I2C1_ReceiveByte(void)
{
    //  int InputLevel[9];
    /*  unsigned char i=8;
      unsigned char ReceiveByte=0;


      SDAH;
      while(i--)
      {
        ReceiveByte<<=1;
        SCLL;
        I2C_delay();
        SCLH;
        I2C_delay();
        if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11)==1)
        {
          ReceiveByte|=0x01;
        }
      }
      SCLL;
      return ReceiveByte;*/

    unsigned char i = 8;
    unsigned char ReceiveByte = 0;
    uint8_t t;
    uint8_t data;

    SDAH();

    while (i--)
    {
        ReceiveByte <<= 1;
        SCLL();
        I2C_delay();
        SCLH();
        /*
        InputLevel[0]=GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11);
        InputLevel[1]=GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11);
        InputLevel[2]=GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11);
        InputLevel[3]=GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11);
        InputLevel[4]=GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11);
        InputLevel[5]=GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11);
        InputLevel[6]=GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11);
        InputLevel[7]=GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11);
        InputLevel[8]=GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11);
        if((InputLevel[0]+InputLevel[1]+InputLevel[2]+InputLevel[3]+InputLevel[4]+InputLevel[5]+InputLevel[6]+InputLevel[7]+InputLevel[8])>=4)
        {
            ReceiveByte|=0x01;
        }
        */
        data = 0;

        for (t = 0; t < 8; t++)
        {
            data += GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11);
        }

        if (data >= 4)
        {
            ReceiveByte |= 0x01;
        }

    }

    SCLL();
    return ReceiveByte;
}

void I2C1_WaitAck(void)
{
    SCLL();
    I2C_delay();
    SDAH();
    I2C_delay();
    SCLH();
    I2C_delay();

    if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11) == 1)
    {
        I2Cerror = 1;
        DEBUG_LEDon();
        I2Cerrorcount++;
    }

    SCLL();

}

/*
//retrun values: 1 - ok; 1 - error;
uint8_t I2C1_CheckAck(void)
{
    uint8_t ack = 0;

      SCLL();
      I2C_delay();
      SDAH();
      I2C_delay();
      SCLH();
      I2C_delay();
      if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11) == 1)
      {
          ack = 1;
          DEBUG_LEDon();
          I2Cerrorcount++;
      }
      SCLL();

    return ack;
}
*/

