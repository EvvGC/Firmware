#include "utils.h"

#include "hw_config.h"
#include "usb_lib.h"
#include "usb_desc.h"
#include "usb_pwr.h"
#include "usb.h"
#include "systick.h"
#include "ringbuffer.h"
#include "hw_config.h"

static tRingBuffer RingBufferUSBTX;
static int USBCallBackCalled=0;
static unsigned long lastCallbackTime;

int usbOverrun(void)
{
	return(RingBufferUSBTX.Overrun);
}


void USBPushTXData(void)
{
	tRingBuffer *rb = &RingBufferUSBTX;
	uint8_t *p = rb->Buffer + rb->Read;
	int len = rb->Write - rb->Read;
	if(len != 0) {
		if(len < 0) {
			len = RingBufferSize(rb) - rb->Read;
		}
	
		len = CDC_Send_DATA(p, len);
		rb->Read = (rb->Read + len) % RingBufferSize(rb);
	}	
}


void USBPushTX(void)
{
	if(packetSent) {// && millis() - lastCallbackTime < 200) {
		return; // transfer will be handled by next callback
	}
	
	// something hangs, retrigger send
	//packetSent = 0; // packetSent is cleared in SetVCPConnectMode() now
	lastCallbackTime = millis();
	USBPushTXData();
}


void EP1_IN_Callback (void)
{
	USBCallBackCalled = 1;
	packetSent = 0;
	lastCallbackTime = millis();
    USBPushTXData();
}


void setupUSB (void) {
  //#define USB_DISC_DEV         GPIOD
  //#define USB_DISC_PIN         11
#if 0
  GPIO_InitTypeDef	GPIO_InitStructure;
 
  GPIO_InitStructure.GPIO_Pin = USB_DISC_PIN;         //LED Output Config
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(USB_DISC_DEV, &GPIO_InitStructure);

  GPIO_ResetBits(USB_DISC_DEV, USB_DISC_PIN); // ala42
  Delay_us(200000);
#endif
  /* setup the apb1 clock for USB */
  //rcc_reg_map *pRCC = RCC_BASE;
  //pRCC->APB1ENR |= RCC_APB1ENR_USBEN;

  /* initialize the usb application */
  //GPIO_SetBits(USB_DISC_DEV, USB_DISC_PIN); // ala42 // presents us to the host
  Set_System();
  Set_USBClock();
  USB_Interrupts_Config();
  USB_Init();
  
  RingBufferInit(&RingBufferUSBTX, &USBPushTX);
}

#if 0
uint32_t usbSendBytes2(const uint8_t* sendBuf, uint32_t len) {
	uint32_t maxLen=32;
	uint32_t sent = 0;

	packetSent=0;
	if(usbIsConnected()) {
		printDirect(" A\r\n");
		unsigned long tStart = micros();
		do {
			uint32_t sendLength = len - sent;
			if(sendLength > maxLen) {
				sendLength = maxLen;
			}
			int lastSend = CDC_Send_DATA((uint8_t*)sendBuf + sent, sendLength);
			if(lastSend > 0) {
				tStart = micros();
			} else {
				if(micros() - tStart > 100) {
					break;
				}
			}
			sent += lastSend;
		} while (sent < len);
	} else {
		printDirect(" B\r\n");
	}
	
	return sent;
}
#endif

uint32_t usbSendBytes(const uint8_t* sendBuf, uint32_t len) {
    if(usbIsConnected()) {
        RingBufferPutBlock(&RingBufferUSBTX, (uint8_t*)sendBuf, len, 0);
    }

    return len;
}

void usbEnableBlockingTx(void) {
	//VCP_SetUSBTxBlocking(1);
}


void usbDisableBlockingTx(void) {
	//VCP_SetUSBTxBlocking(0);
}


uint32_t usbBytesAvailable(void) {
	return receiveLength;
}


uint32_t usbReceiveBytes(uint8_t* recvBuf, uint32_t len) {
	  int newBytes = usbBytesAvailable();
	  if ((int)len > newBytes) {
	      len = newBytes;
	  }

	  CDC_Receive_DATA(recvBuf, len);

	  return len;
}

#if 0
uint8_t usbIsConfigured2(void) {
  return 1; //(bDeviceState == CONFIGURED);
}

uint8_t usbIsConnected2(void) {
  return 1; //(bDeviceState != UNCONNECTED);
}

RESULT usbPowerOn(void) {
  return USB_SUCCESS;
}

RESULT usbPowerOff(void) {
  //USBD_DeInitFull(&USB_OTG_dev);
  return USB_SUCCESS;
}
#endif

void usbDsbISR(void) {};
