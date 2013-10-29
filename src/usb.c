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

#include "utils.h"
#include "hw_config.h"
#include "usb_lib.h"
#include "usb_desc.h"
#include "usb_pwr.h"
#include "usb.h"
#include "systick.h"
#include "ringbuffer.h"
#include "hw_config.h"

tRingBuffer RingBufferUSBTX;
static int USBCallBackCalled = 0;
static unsigned long lastCallbackTime;

int usbOverrun(void)
{
    return (RingBufferUSBTX.Overrun);
}

void USBPushTXData(void)
{
    tRingBuffer *rb = &RingBufferUSBTX;
    uint8_t *p = rb->Buffer + rb->Read;
    int len = rb->Write - rb->Read;

    if (len != 0)
    {
        if (len < 0)
        {
            len = RingBufferSize(rb) - rb->Read;
        }

        len = CDC_Send_DATA(p, len);
        rb->Read = (rb->Read + len) % RingBufferSize(rb);
    }
}

void USBPushTX(void)
{
    if (packetSent) // && millis() - lastCallbackTime < 200) {
    {
        return; // transfer will be handled by next callback
    }

    // something hangs, retrigger send
    //packetSent = 0; // packetSent is cleared in SetVCPConnectMode() now
    lastCallbackTime = millis();
    USBPushTXData();
}

void EP1_IN_Callback(void)
{
    USBCallBackCalled = 1;
    packetSent = 0;
    lastCallbackTime = millis();
    USBPushTXData();
}

void setupUSB(void)
{
    Set_System();
    Set_USBClock();
    USB_Interrupts_Config();
    USB_Init();

    RingBufferInit(&RingBufferUSBTX, &USBPushTX);
}

uint32_t usbSendBytes(const uint8_t *sendBuf, uint32_t len)
{
    if (usbIsConnected())
    {
        RingBufferPutBlock(&RingBufferUSBTX, (uint8_t *)sendBuf, len, 0);
    }

    return len;
}

void usbEnableBlockingTx(void)
{
    //VCP_SetUSBTxBlocking(1);
}

void usbDisableBlockingTx(void)
{
    //VCP_SetUSBTxBlocking(0);
}

uint32_t usbBytesAvailable(void)
{
    return receiveLength;
}

uint32_t usbReceiveBytes(uint8_t *recvBuf, uint32_t len)
{
    int newBytes = usbBytesAvailable();

    if ((int)len > newBytes)
    {
        len = newBytes;
    }

    CDC_Receive_DATA(recvBuf, len);

    return len;
}

void usbDsbISR(void) {};
