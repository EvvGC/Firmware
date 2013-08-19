#include "adc.h"
#include "comio.h"
#include "commhandler.h"
#include "config.h"
#include "fasttrig.h"
#include "engine.h"
#include "gyro.h"
#include "pwm.h"
#include "pins.h"
#include "rc.h"
#include "systick.h"
#include "utils.h"
#include "hw_config.h"

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
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    /* Enable the USARTy Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;//Preemption Priority
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


static float idlePerf;
float GetIdlePerf(void)
{
	return idlePerf;
}


void setup(void)
{
	InitSysTick();
	
    Periph_clock_enable();
    GPIO_Config();

    LEDon();
    Delay_ms(10); //short blink
    LEDoff();
    Delay_ms(50);

	ComInit();
	print("\r\n\r\nEvvGC firmware starting up...\r\n");

	print("init NVIC...\r\n");
    NVIC_Configuration();

	print("init motor PWM...\r\n");
	PWMConfig();

	Delay_ms(2000);
	if(GetVCPConnectMode() != eVCPConnectReset) {
		print("\r\nUSB startup delay...\r\n");
		Delay_ms(3000);
		if(GetVCPConnectMode() == eVCPConnectData) {
			print("\r\n\r\nEvvGC firmware starting up, USB connected...\r\n");
		}
	}
	if ((RCC->CR & RCC_CR_HSERDY) != RESET) {
		print("running on external HSE clock, clock rate is %dMHz\r\n", SystemCoreClock/1000000);
	} else {
		print("ERROR: running on internal HSI clock, clock rate is %dMHz\r\n", SystemCoreClock/1000000);
	}
	
	print("init ADC...\r\n");
	ADC_Config();
	
	print("init MPU6050...\r\n");
    while (MPU6050_Init()) {
		print("init MPU6050 failed, retrying...\r\n");
		Blink();
	}

	print("loading config...\r\n");
    configLoad();
	
	print("calibrating MPU6050...\r\n");
    MPU6050_Gyro_calibration();
	
	print("init RC...\r\n");
    RC_Config();

	
	InitSinArray();

	print("entering main loop...\r\n");

	SysTickAttachCallback(WatchDog);
}


static int GetIdleMax(void)
{
	unsigned int t0 = millis();
	while(millis() == t0)
		;

	unsigned int lastTime = micros();
	int idleLoops=0;
	__disable_irq();
    while(1) {
		idleLoops++;
		unsigned int currentTime = micros();
		unsigned int timePassed = currentTime - lastTime;
		if(timePassed >= 500U) {			
			break;
		}		
    }
	__enable_irq();
	int idleMax = 2*idleLoops; // loops/ms
	idleLoops = 0;
	
	return idleMax;
}


int main(void)
{
    setup();	
	int idleMax = GetIdleMax();

	int idleLoops=0;
	unsigned int lastTime = micros();
    while(1) {
		idleLoops++;
		unsigned int currentTime = micros();
		unsigned int timePassed = currentTime - lastTime;
		if(timePassed >= 2000) {
			idlePerf = idleLoops * 100.0 * 1000/timePassed /idleMax; // perf in percent
			idleLoops = 0;

			if(ConfigMode == 0) {
				engineProcess(timePassed/1000000.0);
			} else {
				PWMOff();
				Blink();
			}
			
			WatchDogCounter = 0;
			CommHandler();			
			lastTime = currentTime;
		}		
    }
}

