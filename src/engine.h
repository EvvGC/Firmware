/*
 *  engine.h
 *
 *  Created on: Jun 26, 2013
 *      Author: Denis aka caat
 */

/*
    Original work Copyright (c) 2012 [Evaldis - RCG user name]
    Modified work Copyright 2012 Alan K. Adamson

    This file is part of EvvGC.

    EvvGC is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    EvvGC is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with EvvGC.  If not, see <http://www.gnu.org/licenses/>.

*/

#ifndef ENGiNE_H_
#define ENGINE_H_

#define PITCH_UP_LIMIT (-50 * D2R)
#define PITCH_DOWN_LIMIT (90 * D2R)
#define CORRECTION_STEP 1.0F

extern int debugPrint;
extern int debugPerf;
extern int debugSense;
extern int debugCnt;
extern int debugRC;
extern int debugOrient;
extern int debugAutoPan;

void Init_Orientation(void);
void engineProcess(float dt);
void Get_Orientation(float *AccAngleSmooth, float *Orient, float *AccData, float *GyroData, float dt);

struct sTraceBuffer
{
	uint32_t	ui32Counter;
    float		fAccX;
    float		fAccY;
    float		fAccZ;
} __attribute__ ((__packed__)) ;
extern int g_bTraceBufferReady;

extern struct sTraceBuffer g_TraceBuffer;

#endif /* ENGINE_H_ */




