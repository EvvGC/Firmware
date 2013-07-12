/*
 * 	config.h
 *
 *	Created on: Jun 25, 2013
 *		Author: Denis caat
 */

#ifndef CONFIG_H_
#define CONFIG_H_
#include <stdint.h>

#define configDataSize 12  // Config data array size
//uint8_t configData[configDataSize]={'1','1','1','1','1','1','1','1','1','1','1','1'};
extern char configData[];
void configLoad(void);
void configSave(void);

#endif /* CONFIG_H_ */
