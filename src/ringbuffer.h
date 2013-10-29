/*
 *  ringbuffer.h
 *
 *  Created on: Aug 1, 2013
 *      Author: ala42
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

#ifndef RINGBUFFER_H_
#define RINGBUFFER_H_

typedef struct
{
    volatile int Read, Write, Overrun;
    unsigned char Buffer[512];
    void (*CallBack)(void);
} tRingBuffer;

void RingBufferInit(tRingBuffer *rb, void (*callback)(void));
int RingBufferSize(tRingBuffer *rb);
int RingBufferFillLevel(tRingBuffer *rb);
void RingBufferPut(tRingBuffer *rb, unsigned char c, int block);
void RingBufferPutBlock(tRingBuffer *rb, unsigned char *data, int dataLen, int block);
int RingBufferGet(tRingBuffer *rb);
int RingBufferPeek(tRingBuffer *rb);

#endif /* RINGBUFFER_H_ */
