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

#ifndef _USBF4_H_
#define _USBF4_H_

#ifdef __cplusplus
extern "C" {
#endif

//#include "usb_conf.h"

void setupUSB(void);
void disableUSB(void);
void usbSuspend(void);
void usbResumeInit(void);
//void usbResume(RESUME_STATE);

//RESULT usbPowerOn(void);
//RESULT usbPowerOff(void);

void usbDsbISR(void);
void usbEnbISR(void);

void   usbBlockingSendByte(char ch);
uint32_t usbSendBytes(const uint8_t *sendBuf, uint32_t len);
uint32_t usbBytesAvailable(void);
uint32_t usbReceiveBytes(uint8_t *recvBuf, uint32_t len);
uint8_t usbGetDTR(void);
uint8_t usbGetRTS(void);
uint8_t usbIsConnected(void);
uint8_t usbIsConfigured(void);
uint16_t usbGetPending(void);
void usbEnableBlockingTx(void);
void usbDisableBlockingTx(void);
int usbOverrun(void);


void __irq_OTG_FS_IRQHandler(void);

#ifdef __cplusplus
} // extern "C"
#endif

#endif // _USB_H_
