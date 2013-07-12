/*
 * 	config.c
 *
 *	Created on: Jun 25, 2013
 *		Author: Denis caat
 */

#include "config.h"
#include "utils.h"
#include "pins.h"
#include "eeprom.h"

//extern uint8_t configData[];

void configLoad(void)
{
    uint8_t i;
    uint8_t data;

    //reads configuration from eeprom
    for (i = 0; i < configDataSize; i++)
    {
        data = ReadFromEEPROM(i);
        configData[i] = data;
        Delay_ms(5);
    }
}

void configSave(void)
{
    uint8_t i;

    LEDon;

    for (i = 0; i < configDataSize; i++)
    {
        //read data from eeprom,
        //check, if it has changed, only then rewrite;
        WriteToEEPROM(i, configData[i]);
        Delay_ms(5);
    }

    LEDoff;
}
