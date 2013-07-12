/*
 * 	engine.h
 *
 *	Created on: Jun 26, 2013
 *		Author: Denis aka caat
 */

#ifndef ENGiNE_H_
#define ENGINE_H_
#include <stdint.h>
extern char configData[];
extern int ConfigMode, stop, watchcounter, rc3a, rc3b, rc3, rc4a, rc4b, rc4;

void engineProcess(void);
void pitch_PID(void);
void roll_PID(void);
void yaw_PID(void);

#endif /* ENGINE_H_ */
