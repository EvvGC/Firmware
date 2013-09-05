/*
 *  ringbuffer.c
 *
 *  Created on: Aug 1, 2013
 *      Author: ala42
 */
#include <stdint.h>
#include <string.h>
#include "ringbuffer.h"

void RingBufferInit(tRingBuffer *rb, void (*callback)(void))
{
    rb->Read = 0;
    rb->Write = 0;
    rb->Overrun = 0;
    rb->CallBack = callback;
}

int RingBufferSize(tRingBuffer *rb)
{
    return sizeof(rb->Buffer);
}

int RingBufferFillLevel(tRingBuffer *rb)
{
    return (rb->Write - rb->Read + RingBufferSize(rb)) % RingBufferSize(rb);
}

void RingBufferPut(tRingBuffer *rb, unsigned char c, int block)
{
    if (block)
    {
        while (RingBufferFillLevel(rb) + 1 == RingBufferSize(rb))
        {
            // wait
        }
    }
    else
    {
        if (RingBufferFillLevel(rb) + 1 == RingBufferSize(rb))
        {
            rb->Overrun++;
            return;
        }
    }

    rb->Buffer[rb->Write] = c;

    if (rb->Write + 1 == RingBufferSize(rb))
    {
        rb->Write = 0;
    }
    else
    {
        rb->Write++;
    }

    if (rb->CallBack)
    {
        rb->CallBack();
    }
}

void RingBufferPutBlock(tRingBuffer *rb, unsigned char *data, int dataLen, int block)
{
    if (block)
    {
        while (RingBufferFillLevel(rb) + dataLen >= RingBufferSize(rb))
        {
            // wait
        }
    }
    else
    {
        if (RingBufferFillLevel(rb) + dataLen >= RingBufferSize(rb))
        {
            rb->Overrun += dataLen;

            if (rb->CallBack)
            {
                rb->CallBack();
            }

            return;
        }
    }

    int free1 = RingBufferSize(rb) - rb->Write;

    if (dataLen <= free1)
    {
        memcpy(rb->Buffer + rb->Write, data, dataLen);

        if (rb->Write + dataLen == RingBufferSize(rb))
        {
            rb->Write = 0;
        }
        else
        {
            rb->Write += dataLen;
        }
    }
    else
    {
        memcpy(rb->Buffer + rb->Write, data, free1);
        int len2 = dataLen - free1;
        memcpy(rb->Buffer, data + free1, len2);
        rb->Write = len2;
    }

    if (rb->CallBack)
    {
        rb->CallBack();
    }
}

int RingBufferGet(tRingBuffer *rb)
{
    if (rb->Read == rb->Write)
    {
        return -1;
    }
    else
    {
        int c = rb->Buffer[rb->Read];

        if (rb->Read + 1 == RingBufferSize(rb))
        {
            rb->Read = 0;
        }
        else
        {
            rb->Read++;
        }

        return c;
    }
}

int RingBufferPeek(tRingBuffer *rb)
{
    if (rb->Read == rb->Write)
    {
        return -1;
    }
    else
    {
        int c = rb->Buffer[rb->Read];

        return c;
    }
}
