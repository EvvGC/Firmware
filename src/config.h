/*
 *  config.h
 *
 *  Created on: Jun 25, 2013
 *      Author: Denis caat
 */

#ifndef CONFIG_H_
#define CONFIG_H_

#define CONFIGDATASIZE 12  // Config data array size
#define LARGEST_CONFIGDATA 254
extern char configData[CONFIGDATASIZE];

void configLoad(void);
void configSave(void);
void printConfig(void);
#endif /* CONFIG_H_ */
