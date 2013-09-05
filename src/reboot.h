/*
 *  reboot.h
 *
 *  Created on: Aug 28, 2013
 *      Author: ala42
 */

#ifndef REBOOT_H_

void BKPInit(void);
unsigned long BKPRead(void);
void BKPWrite(unsigned long val);

void reboot(void);
void bootloader(void);

#endif /* REBOOT_H_ */
