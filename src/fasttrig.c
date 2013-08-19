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
		int ix = ((int)(x/M_TWOPI*(float)SINARRAYSIZE)) % SINARRAYSIZE;
		return sinDataI16[ix]/(float)SINARRAYSCALE;
	} else {
		int ix = ((int)(-x/M_TWOPI*(float)SINARRAYSIZE)) % SINARRAYSIZE;
		return -sinDataI16[ix]/(float)SINARRAYSCALE;
	}
}