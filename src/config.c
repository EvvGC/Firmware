/*
 *  config.c
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

#include <stdint.h>
#include "config.h"
#include "utils.h"
//#include "pins.h"
#include "eeprom.h"
#include "comio.h"

char configData[CONFIGDATASIZE] = {49, 50, 51, 97, 98, 99, 51, 52, 53, '0', '1', 54};

void configLoad(void)
{
    //reads configuration from eeprom
    for (int i = 0; i < CONFIGDATASIZE; i++)
    {
        int data = ReadFromEEPROM(i);

        if (data >= 0 && data <= LARGEST_CONFIGDATA)
        {
            configData[i] = data;
        }

        Delay_ms(5);
    }
}

void configSave(void)
{
    uint8_t i;

    LEDon();

    for (i = 0; i < CONFIGDATASIZE; i++)
    {
        //read data from eeprom,
        //check, if it has changed, only then rewrite;
        int data = ReadFromEEPROM(i);

        if (data != -1 && data != configData[i])
        {
            WriteToEEPROM(i, configData[i]);
        }

        Delay_ms(5);
    }

    LEDoff();
}

void printConfig(void)
{
    print("\r\nconfig data: ");

    for (int i = 0; i < CONFIGDATASIZE; i++)
    {
        uint8_t data = configData[i]; // ala42
        print(" %02X ", data);
    }

    print("\r\n");
}
