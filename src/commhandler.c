/*
 *  commhandler.h
 *
 *  Created on: Aug 18, 2013
 *      Author: ala42
 *
 *  extracted code from main.c
 */
#include <stdint.h>
#include "main.h"
#include "config.h"
#include "utils.h"
#include "eeprom.h"
#include "engine.h"
#include "commhandler.h"
#include "pwm.h"
#include "systick.h"
#include "comio.h"
#include "hw_config.h"
#include "reboot.h"

int ConfigMode;

void CommHandler(void) //UART4 Interrupt handler implementation
{
    int c = GetChar();
    extern int bDeviceState;

    if (c >= 0)
    {
        //ConfigMode = 1;
        LEDon();
        //print("got char %02X\r\n", c);

        switch (c)
        {
            case 'a':
                debugAutoPan ^= 1;
                print("Autopan messages %s\r\n", debugAutoPan ? "on" : "off");
                break;

            case 'b':
                print("rebooting into boot loader ...\r\n");
                Delay_ms(1000);
                bootloader();
                break;

            case 'c':
                debugCnt ^= 1;
                print("counter messages %s\r\n", debugCnt ? "on" : "off");
                break;

            case 'd':
                debugPrint ^= 1;
                print("debug messages %s\r\n", debugPrint ? "on" : "off");
                break;

            case 'g':
                Delay_ms(100);
                PutChar('x');

                for (int i = 0; i < CONFIGDATASIZE; i++)
                {
                    uint8_t data = configData[i];
                    PutChar(data);
                }

                break;

            case 'G':
                printConfig();
                break;

#if 0

            case 'H':
                if (CharAvailable() >= CONFIGDATASIZE)
                {
                    for (int i = 0; i < CONFIGDATASIZE; i++)
                    {
                        uint8_t data = GetChar();

                        if (data <= LARGEST_CONFIGDATA)
                        {
                            configData[i] = data;
                        }
                    }

                    configSave();
                }
                else
                {
                    UnGetChar(c); // try again in next loop
                }

                break;
#endif

            case 'h':
                for (int i = 0; i < CONFIGDATASIZE; i++)
                {
                    int data;

                    while ((data = GetChar()) < 0)
                        ;

                    if (data <= LARGEST_CONFIGDATA)
                    {
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

            case 'o':
                debugOrient ^= 1;
                print("Orientation messages %s\r\n", debugOrient ? "on" : "off");
                break;

            case 'p':
                debugPerf ^= 1;
                print("performance messages %s\r\n", debugPerf ? "on" : "off");
                break;

            case 'r':
                debugRC ^= 1;
                print("RC messages %s\r\n", debugRC ? "on" : "off");
                break;

            case 'R':
                print("rebooting...\r\n");
                Delay_ms(1000);
                reboot();
                break;

            case 's':
                debugSense ^= 1;
                print("Sensor messages %s\r\n", debugSense ? "on" : "off");
                break;

            case 'u':
            {
                extern int bDeviceState;
                printUSART("\r\nYY bDeviceState %3d  VCPConnectMode %d\r\n", bDeviceState, GetVCPConnectMode());
                break;
            }

            case 'v':
                print("Version: %s\r\n", __EV_VERSION);
                break;

            case '+':
                testPhase += 0.1;
                print("test phase output %5.1f\r\n", testPhase);
                break;

            case '-':
                testPhase -= 0.1;
                print("test phase output %5.1f\r\n", testPhase);
                break;

            case '?':
                print("CLI documentation\r\n");
                print("\t'+' test phase output increase (now %5.1f)\r\n", testPhase);
                print("\t'-' test phase output decrease (now %5.1f)\r\n", testPhase);
                print("\t'a' autopan messages display (now %s)\r\n", debugAutoPan ? "on" : "off");
                print("\t'b' reboot into bootloader\r\n");
                print("\t'c' counter messages display (now %s)\r\n", debugCnt ? "on" : "off");
                print("\t'd' debug messages display (now %s)\r\n", debugPrint ? "on" : "off");
                print("\t'g' dump configuration (binary)\r\n");
                print("\t'G' dump configuration (hexadecimal)\r\n");
                print("\t'h' write and save config array\r\n");
                print("\t'i' enter config mode (now %s)\r\n", ConfigMode ? "on" : "off");
                print("\t'j' leave config mode (now %s)\r\n", ConfigMode ? "on" : "off");
                print("\t'o' orientation messages display (now %s)\r\n", debugOrient ? "on" : "off");
                print("\t'p' performance messages display (now %s)\r\n", debugPerf ? "on" : "off");
                print("\t'r' RC messages display (now %s)\r\n", debugRC ? "on" : "off");
                print("\t'R' reboot\r\n");
                print("\t's' toggle sensor messages display (now %s)\r\n", debugSense ? "on" : "off");
                print("\t'u' print USB state (bDeviceState %3d  VCPConnectMode %d)\r\n", bDeviceState, GetVCPConnectMode());
                print("\t'v' print version (%s)\r\n", __EV_VERSION);
            	break;

            default:
                // TODO
                break;
        }
    }
}
