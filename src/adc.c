/*
 *  adc.c
 *
 *  Created on: Jun 25, 2013
 *      Author: Denis aka caat
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

#include <stdint.h>
#include "adc.h"
#include "stm32f10x.h"

void ADC_Config(void)
{
    GPIO_InitTypeDef    GPIO_InitStructure;
    ADC_InitTypeDef  ADC_InitStructure;
    // Configure ADC on ADC123_IN13    pin PC3
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOC, &GPIO_InitStructure);


    /* PCLK2 is the APB2 clock */
    /* ADCCLK = PCLK2/6 = 72/6 = 12MHz*/
    RCC_ADCCLKConfig(RCC_PCLK2_Div6);

    /* Enable ADC1 clock so that we can talk to it */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    /* Put everything back to power-on defaults */
    ADC_DeInit(ADC1);

    /* ADC1 Configuration ------------------------------------------------------*/
    /* ADC1 and ADC2 operate independently */
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    /* Disable the scan conversion so we do one at a time */
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    /* Don't do continuous conversions - do them on demand */
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    /* Start conversion by software, not an external trigger */
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    /* Conversions are 12 bit - put them in the lower 12 bits of the result */
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    /* Say how many channels would be used by the sequencer */
    ADC_InitStructure.ADC_NbrOfChannel = 1;

    /* Now do the setup */
    ADC_Init(ADC1, &ADC_InitStructure);
    /* Enable ADC1 */
    ADC_Cmd(ADC1, ENABLE);

    /* Enable ADC1 reset calibration register */
    ADC_ResetCalibration(ADC1);

    /* Check the end of ADC1 reset calibration register */
    while (ADC_GetResetCalibrationStatus(ADC1));

    /* Start ADC1 calibration */
    ADC_StartCalibration(ADC1);

    /* Check the end of ADC1 calibration */
    while (ADC_GetCalibrationStatus(ADC1));


}

//u16 readADC1(u8 channel)
uint16_t readADC1(uint8_t channel)
{
    ADC_RegularChannelConfig(ADC1, channel, 1, ADC_SampleTime_28Cycles5);
    // Start the conversion
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);

    // Wait until conversion completion
    while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);

    // Get the conversion value
    return ADC_GetConversionValue(ADC1);
}
