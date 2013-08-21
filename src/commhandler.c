/*
 * 	commhandler.h
 *
 *	Created on: Aug 18, 2013
 *		Author: ala42
 *
 *  extracted code from main.c
 */

#include "config.h"
#include "utils.h"
#include "eeprom.h"
#include "engine.h"
#include "pwm.h"
#include "systick.h"
#include "comio.h"
#include "hw_config.h"

int ConfigMode;


void CommHandler(void) //UART4 Interrupt handler implementation
{
    int c = GetChar();
	if(c >= 0) {
		//ConfigMode = 1;
		LEDon();
		//print("got char %02X\r\n", c);
		
		switch(c) {
			case 'g':
				Delay_ms(100);
				PutChar('x');

				for (int i = 0; i < CONFIGDATASIZE; i++)	{
					uint8_t data = configData[i];
					PutChar(data);
				}
				break;
			case 'G':
				for (int i = 0; i < CONFIGDATASIZE; i++)	{
					uint8_t data = configData[i]; // ala42
					print("  %2d  %02X %3d\r\n", i, data, data);
				}
				break;
			case 'h':
				for(int i=0; i < CONFIGDATASIZE; i++) {
					int data;
					while((data = GetChar()) < 0)
						;
					if(data <= 100) {
						configData[i] = data;
					}
				}
				configSave();
				break;
			case 'i':
				ConfigMode = 1;
				break;
			case 'j':
				ConfigMode = 0;
				break;
			case '+':
				testPhase += 0.1;
				print("test phase output %5.1f\r\n", testPhase);
				break;
			case '-':
				testPhase -= 0.1;
				print("test phase output %5.1f\r\n", testPhase);
				break;
			case 'd':
				debugPrint ^= 1;
				print("debug messages %s\r\n", debugPrint ? "on" : "off");
				break;
			case 'p':
				debugPerf ^= 1;
				print("performance messages %s\r\n", debugPerf ? "on" : "off");
				break;
			case 'c':
				debugCnt ^= 1;
				print("counter messages %s\r\n", debugCnt ? "on" : "off");
				break;
				
			case 'u':
			{
				extern int bDeviceState;
				printUSART("\r\nYY bDeviceState %3d  VCPConnectMode %d\r\n", bDeviceState, GetVCPConnectMode());
				break;
			}
				
		}
    }
}
