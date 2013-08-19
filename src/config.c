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

char configData[CONFIGDATASIZE] = {10, 10, 10, 50, 50, 50, 50, 50, 50, '1', '1', 50};

void configLoad(void)
{
    //reads configuration from eeprom
    for (int i = 0; i < CONFIGDATASIZE; i++)
    {
        int data = ReadFromEEPROM(i);
		if(data >= 0 && data <= 100) {
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
		if(data != -1 && data != configData[i]) {
			WriteToEEPROM(i, configData[i]);
		}
        Delay_ms(5);
    }

    LEDoff();
}
