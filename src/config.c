/*
 * 	config.c
 *
 *	Created on: Jun 25, 2013
 *		Author: Denis caat
 */

#include "config.h"
#include "utils.h"
//#include "pins.h"
#include "eeprom.h"
#include "comio.h"

char configData[configDataSize] = {10, 10, 10, 50, 50, 50, 50, 50, 50, '1', '1', 50};

void configLoad(void)
{
    //reads configuration from eeprom
    for (int i = 0; i < configDataSize; i++)
    {
        int data = ReadFromEEPROM(i);
		if(data >= 0) {
			configData[i] = data;
		}
        Delay_ms(5);
    }
}

void configSave(void)
{
    uint8_t i;
	DEBUG_PutChar('C');

    LEDon();

    for (i = 0; i < configDataSize; i++)
    {
        //read data from eeprom,
        //check, if it has changed, only then rewrite;
        WriteToEEPROM(i, configData[i]);
        Delay_ms(5);
    }

    LEDoff();
}
