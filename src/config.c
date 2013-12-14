/*
 *  config.c
 *
 *  Created on: Jun 25, 2013
 *      Author: Denis caat
 */
#include <stdint.h>
#include "config.h"
#include "utils.h"
//#include "pins.h"
#include "eeprom.h"
#include "comio.h"

char configData[CONFIGDATASIZE] = {40, 85, 60, 30, 55, 40, 35, 55, 60, '0', '0', 64};

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
