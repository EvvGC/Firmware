/*
 * 	fasttrig.c
 *
 *	Created on: Aug 10, 2013
 *		Author: ala42
 */
#include "fasttrig.h"
#include <math.h>
#include "utils.h"

short int sinDataI16[SINARRAYSIZE];

void InitSinArray(void)
{
	for(int i=0; i<SINARRAYSIZE; i++) {
		float x = i*M_TWOPI/SINARRAYSIZE;
		sinDataI16[i] = (short int)Round(sinf(x) * SINARRAYSCALE);	
		//print("i %3d  x %f  sin %d\r\n", i, x, (int)sinDataI16[i]);
	}
}

float fastSin(float x)
{
	if(x >= 0) {
		int ix = ((int)(x/M_TWOPI*1024.0F)) % 1024;
		return sinDataI16[ix]/32767.0F;
	} else {
		int ix = ((int)(-x/M_TWOPI*1024.0F)) % 1024;
		return -sinDataI16[ix]/32767.0F;
	}
}