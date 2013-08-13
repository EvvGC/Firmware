/*
 * 	eeprom.h
 *
 *	Created on: Jun 25, 2013
 *		Author: Denis aka caat
 */

#ifndef EEPROM_H_
#define EEPROM_H_
#include <stdint.h>

void WriteToEEPROM(uint8_t addressToWrite, uint8_t DataToWrite);
int ReadFromEEPROM(uint8_t readAddr);

#endif /* EEPROM_H_ */
