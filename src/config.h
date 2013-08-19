/*
 * 	config.h
 *
 *	Created on: Jun 25, 2013
 *		Author: Denis caat
 */

#ifndef CONFIG_H_
#define CONFIG_H_
#include <stdint.h>

#define CONFIGDATASIZE 12  // Config data array size
extern char configData[CONFIGDATASIZE];
void configLoad(void);
void configSave(void);

#endif /* CONFIG_H_ */
