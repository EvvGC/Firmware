#include "config.h"
#include "pins.h"
#include "usart.h"
#include "utils.h"
#include "adc.h"
#include "eeprom.h"
#include "engine.h"
#include "gyro.h"
#include "pwm.h"
#include "systick.h"
#include "rc.h"
#include "comio.h"
#include "fasttrig.h"

//#include "sys/printf.h"

static int ConfigMode;
static volatile int WatchDogCounter;

void Periph_clock_enable(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
                           RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD |
                           RCC_APB2Periph_GPIOE | RCC_APB2Periph_AFIO |
                           RCC_APB2Periph_ADC1	| RCC_APB2Periph_TIM1 |
                           RCC_APB2Periph_TIM8, ENABLE);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5  | RCC_APB1Periph_TIM2 |
                           RCC_APB1Periph_UART4 | RCC_APB1Periph_TIM3 |
                           RCC_APB1Periph_TIM4, ENABLE);

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,  ENABLE);
}


void NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Configure the NVIC Preemption Priority Bits */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);

    /* Enable the USARTy Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

#include "stm32f10x_tim.h"


void WatchDog(void)
{
    if (WatchDogCounter++ > 1000)
    {
		LEDtoggle();
        PWMOff();
		DEBUG_PutChar('W');
		WatchDogCounter = 0;
    }
}


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

				for (int i = 0; i < configDataSize; i++)	{
					uint8_t data = ReadFromEEPROM(i);
					data = configData[i]; // ala42
					Delay_ms(1);
					PutChar(data);
				}
				break;
			case 'G':
				print("configDataSize %d\r\n", configDataSize);

				for (int i = 0; i < configDataSize; i++)	{
					uint8_t data = ReadFromEEPROM(i);
					data = configData[i]; // ala42
					print("  %2d  %02X %3d\r\n", i, data, data);
				}
				break;
			case 'h':
				if (CharAvailable() >= configDataSize) {
					for(int i=0; i<configDataSize; i++) {
						configData[i] = GetChar();						
					}
					configSave();
				} else {
					UnGetChar(c); // try again in next loop
				}
				break;
			case 'i':
				ConfigMode = 1;
				break;
			case 'j':
				ConfigMode = 0;
				break;
			case '+':
				testPhase += 1.0;
				print("test phase output %5.1f\r\n", testPhase);
				break;
			case '-':
				testPhase -= 1.0;
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
		}
    }
}


int main(void)
{
	InitSysTick();
	
    Periph_clock_enable();
    GPIO_Config();

    LEDon();
    Delay_ms(10); //short blink
    LEDoff();
    Delay_ms(50);

    Usart4Init();
	DEBUG_PutString("EvvGC firmware starting up...\r\n");

	DEBUG_PutString("init NVIC...\r\n");
    NVIC_Configuration();
	
	if ((RCC->CR & RCC_CR_HSERDY) != RESET) {
		print("running on external HSE clock, clock rate is %dMHz\r\n", SystemCoreClock/1000000);
	} else {
		print("ERROR: running on internal HSI clock, clock rate is %dMHz\r\n", SystemCoreClock/1000000);
	}
	
	DEBUG_PutString("init ADC...\r\n");
	ADC_Config();
	
	DEBUG_PutString("init MPU6050...\r\n");
    while (MPU6050_Init()) {
		DEBUG_PutString("init MPU6050 failed, retrying...\r\n");
		Blink();
	}

	DEBUG_PutString("loading config...\r\n");
    configLoad();
	
	DEBUG_PutString("calibrating MPU6050...\r\n");
    MPU6050_Gyro_calibration();
	
	DEBUG_PutString("init RC...\r\n");
    RC_Config();

	DEBUG_PutString("init motor PWM...\r\n");
	PWMConfig();
	
	InitSinArray();
	DEBUG_PutString("entering main loop...\r\n");

	SysTickAttachCallback(WatchDog);
	
	unsigned int lastTime = micros();
    while (1)
    {
		unsigned int currentTime = micros();
		unsigned int timePassed = currentTime - lastTime;
		if(timePassed >= 2000) {
			if(ConfigMode == 0) {
				engineProcess(timePassed/1000000.0);
			} else {
				PWMOff();
				Blink();
			}
			lastTime = currentTime;
			WatchDogCounter = 0;
		}
		
		CommHandler();
    }
}

