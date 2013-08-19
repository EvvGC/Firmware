/*
 * 	ringbuffer.h
 *
 *	Created on: Aug 1, 2013
 *		Author: ala42
 */

#ifndef RINGBUFFER_H_
#define RINGBUFFER_H_

#include <stdint.h>

typedef struct {
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
