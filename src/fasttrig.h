/*
 * 	fasttrig.h
 *
 *	Created on: Aug 10, 2013
 *		Author: ala42
 */

#ifndef FASTTRIG_H_
#define FASTTRIG_H_

//#include <stdint.h>
#define SINARRAYSIZE 1024
#define SINARRAYSCALE 32767

extern short int sinDataI16[];
void InitSinArray(void);
float fastSin(float x);

#endif /* FASTTRIG_H_ */
