/*
  October 2012

  aq32Plus Rev -

  Copyright (c) 2012 John Ihlein.  All rights reserved.

  Open Source STM32 Based Multicopter Controller Software

  Includes code and/or ideas from:

  1)AeroQuad
  2)BaseFlight
  3)CH Robotics
  4)MultiWii
  5)S.O.H. Madgwick
  6)UAVX

  Designed to run on the AQ32 Flight Control Board

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

///////////////////////////////////////////////////////////////////////////////

#include "board.h"

///////////////////////////////////////////////////////////////////////////////

// I2C1
// SCL  PB6
// SDA  PB7

// I2C2
// SCL  PB10
// SDA  PB11

///////////////////////////////////////////////////////////////////////////////
// I2C Defines and Variables
///////////////////////////////////////////////////////////////////////////////

#define I2C1_GPIO            GPIOB
#define I2C1_SCL_PIN         GPIO_Pin_6
#define I2C1_SCL_PIN_SOURCE  GPIO_PinSource6
#define I2C1_SDA_PIN         GPIO_Pin_7
#define I2C1_SDA_PIN_SOURCE  GPIO_PinSource7

#define I2C2_GPIO            GPIOB
#define I2C2_SCL_PIN         GPIO_Pin_10
#define I2C2_SCL_PIN_SOURCE  GPIO_PinSource10
#define I2C2_SDA_PIN         GPIO_Pin_11
#define I2C2_SDA_PIN_SOURCE  GPIO_PinSource11

#define I2C_DEFAULT_TIMEOUT 30000

static I2C_TypeDef *I2Cx;

static volatile uint16_t i2c1ErrorCount = 0;
static volatile uint16_t i2c2ErrorCount = 0;

static volatile bool busy;

static volatile uint8_t addr;
static volatile uint8_t reg;
static volatile uint8_t bytes;
static volatile uint8_t writing;
static volatile uint8_t reading;
static volatile uint8_t *write_p;
static volatile uint8_t *read_p;

///////////////////////////////////////////////////////////////////////////////
// I2C Error Handler
///////////////////////////////////////////////////////////////////////////////

void I2C_ER_Handler(void)
{
    volatile uint32_t SR1Register, SR2Register;

    SR1Register = I2Cx->SR1;                                              // Read the I2Cx status register

    if (SR1Register & (I2C_SR1_AF   |
                       I2C_SR1_ARLO |
                       I2C_SR1_BERR ))                                    // If AF, BERR or ARLO, abandon the current job and commence new if there are jobs
    {
        SR2Register = I2Cx->SR2;                                          // Read second status register to clear ADDR if it is set (note that BTF will not be set after a NACK)
        I2C_ITConfig(I2Cx, I2C_IT_BUF, DISABLE);                          // Disable the RXNE/TXE interrupt - prevent the ISR tailchaining onto the ER (hopefully)
        if (!(SR1Register & I2C_SR1_ARLO) && !(I2Cx->CR1 & I2C_CR1_STOP)) // If we dont have an ARLO error, ensure sending of a stop
        {
            if (I2Cx->CR1 & I2C_CR1_START)                                // We are currently trying to send a start, this is very bad as start,stop will hang the peripheral
            {
                while (I2Cx->CR1 & I2C_CR1_START);                        // Wait for any start to finish sending
                I2C_GenerateSTOP(I2Cx, ENABLE);                           // Send stop to finalise bus transaction
                while (I2Cx->CR1 & I2C_CR1_STOP);                         // Wait for stop to finish sending
                i2cInit(I2Cx);                                            // Reset and configure the hardware
            } else
            {
                I2C_GenerateSTOP(I2Cx, ENABLE);                           // Stop to free up the bus
                I2C_ITConfig(I2Cx, I2C_IT_EVT | I2C_IT_ERR, DISABLE);     // Disable EVT and ERR interrupts while bus inactive
            }
        }
    }

    I2Cx->SR1 &= ~(I2C_SR1_OVR  |
                   I2C_SR1_AF   |
                   I2C_SR1_ARLO |
                   I2C_SR1_BERR );                                        // Reset all the error bits to clear the interrupt

    busy = 0;
}

///////////////////////////////////////////////////////////////////////////////
// I2C Event Handler
///////////////////////////////////////////////////////////////////////////////

void I2C_EV_Handler(void)
{
    static uint8_t subaddress_sent, final_stop;                         // Flag to indicate if subaddess sent, flag to indicate final bus condition
    static int8_t index;                                                // Index is signed -1==send the subaddress

    uint8_t SReg_1 = I2Cx->SR1;                                         // Read the status register here

    if (SReg_1 & I2C_SR1_SB)                                            // We just sent a start - EV5 in ref manual
    {
        I2Cx->CR1 &= ~I2C_CR1_POS;                                      // Reset the POS bit so ACK/NACK applied to the current byte
        I2C_AcknowledgeConfig(I2Cx, ENABLE);                            // Make sure ACK is on
        index = 0;                                                      // Reset the index

        if (reading && (subaddress_sent || 0xFF == reg))                // We have sent the subaddr or no subaddress to send
        {
            subaddress_sent = 1;                                        // Make sure this is set in case of no subaddress, so following code runs correctly
            if (bytes == 2)
                I2Cx->CR1 |= I2C_CR1_POS;                               // Set the POS bit so NACK applied to the final byte in the two byte read
            I2C_Send7bitAddress(I2Cx, addr, I2C_Direction_Receiver);    // Send the address and set hardware mode
        }
        else                                                            // Direction is Tx, or we havent sent the sub and rep start
        {
            I2C_Send7bitAddress(I2Cx, addr, I2C_Direction_Transmitter); // Send the address and set hardware mode
            if (reg != 0xFF)                                            // 0xFF as subaddress means it will be ignored, in Tx or Rx mode
                index = -1;                                             // Send a subaddress
        }
    }
    else if (SReg_1 & I2C_SR1_ADDR)                                     // We just sent the address - EV6 in ref manual
    {
        #pragma GCC diagnostic push
        #pragma GCC diagnostic ignored "-Wunused-but-set-variable"

        volatile uint8_t a;                                             // Read SR1, SR2 to clear ADDR

        #pragma GCC diagnostic pop

        __DMB(); // memory fence to control hardware
        if (bytes == 1 && reading && subaddress_sent)                   // We are receiving 1 byte - EV6_3
        {
            I2C_AcknowledgeConfig(I2Cx, DISABLE);                       // Turn off ACK
            __DMB();
            a = I2Cx->SR2;                                              // Clear ADDR after ACK is turned off
            I2C_GenerateSTOP(I2Cx, ENABLE);                             // Program the stop
            final_stop = 1;
            I2C_ITConfig(I2Cx, I2C_IT_BUF, ENABLE);                     // Allow us to have an EV7
        }
        else                                                            // EV6 and EV6_1
        {
            a = I2Cx->SR2;                                              // Clear the ADDR here
            __DMB();
            if (bytes == 2 && reading && subaddress_sent)               // Rx 2 bytes - EV6_1
            {
                I2C_AcknowledgeConfig(I2Cx, DISABLE);                   // Turn off ACK
                I2C_ITConfig(I2Cx, I2C_IT_BUF, DISABLE);                // Disable TXE to allow the buffer to fill
            }
            else if (bytes == 3 && reading && subaddress_sent)          // Rx 3 bytes
                I2C_ITConfig(I2Cx, I2C_IT_BUF, DISABLE);                // Make sure RXNE disabled so we get a BTF in two bytes time
            else                                                        // Receiving greater than three bytes, sending subaddress, or transmitting
                I2C_ITConfig(I2Cx, I2C_IT_BUF, ENABLE);
        }
    }
    else if (SReg_1 & I2C_SR1_BTF)                                      // Byte transfer finished - EV7_2, EV7_3 or EV8_2
    {
        final_stop = 1;
        if (reading && subaddress_sent)                                 // EV7_2, EV7_3
        {
            if (bytes > 2)                                              // EV7_2
            {
                I2C_AcknowledgeConfig(I2Cx, DISABLE);                   // Turn off ACK
                read_p[index++] = I2C_ReceiveData(I2Cx);                // Read data N-2
                I2C_GenerateSTOP(I2Cx, ENABLE);                         // Program the Stop
                final_stop = 1;                                         // Required to fix hardware
                read_p[index++] = I2C_ReceiveData(I2Cx);                // Read data N-1
                I2C_ITConfig(I2Cx, I2C_IT_BUF, ENABLE);                 // Enable RxNE to allow the final EV7 to read data N
            }
            else                                                        // EV7_3
            {
                if (final_stop)
                    I2C_GenerateSTOP(I2Cx, ENABLE);                     // Program the Stop
                else
                    I2C_GenerateSTART(I2Cx, ENABLE);                    // Program a rep start

                read_p[index++] = I2C_ReceiveData(I2Cx);                // Read data N-1
                read_p[index++] = I2C_ReceiveData(I2Cx);                // Read data N
                index++;                                                // To show job completed
            }
        }
        else                                                            // EV8_2, which may be due to a subaddress sent or a write completion
        {
            if (subaddress_sent || (writing)) {
                if (final_stop)
                    I2C_GenerateSTOP(I2Cx, ENABLE);                     // Program the Stop
                else
                    I2C_GenerateSTART(I2Cx, ENABLE);                    // Program a rep start

                index++;                                                // To show that the job is complete
            }
            else                                                        // We need to send a subaddress
            {
                I2C_GenerateSTART(I2Cx, ENABLE);                        // Program the repeated Start
                subaddress_sent = 1;                                    // This is set back to zero upon completion of the current task
            }
        }
        while (I2Cx->CR1 & I2C_CR1_START) { ; }                         // We must wait for the start to clear, otherwise we get constant BTF
    }
    else if (SReg_1 & I2C_SR1_RXNE)                                     // Byte received - EV7
    {
        read_p[index++] = I2C_ReceiveData(I2Cx);
        if (bytes == (index + 3))
            I2C_ITConfig(I2Cx, I2C_IT_BUF, DISABLE);                    // Disable RxNE to allow the buffer to flush so we can get an EV7_2

        if (bytes == index)                                             // We have completed a final EV7
            index++;                                                    // To show job is complete
    }
    else if (SReg_1 & I2C_SR1_TXE)                                      // Byte transmitted - EV8/EV8_1
    {
        if (index != -1)                                                // We dont have a subaddress to send
        {
            I2C_SendData(I2Cx, write_p[index++]);
            if (bytes == index)                                         // We have sent all the data
                I2C_ITConfig(I2Cx, I2C_IT_BUF, DISABLE);                // Disable TXE to allow the buffer to flush
        }
        else
        {
            index++;
            I2C_SendData(I2Cx, reg);                                    // Send the subaddress
            if (reading || !bytes)                                      // If receiving or sending 0 bytes, flush now
                I2C_ITConfig(I2Cx, I2C_IT_BUF, DISABLE);                // Disable TXE to allow the buffer to flush
        }
    }
    if (index == bytes + 1)                                             // We have completed the current job
    {
        subaddress_sent = 0;                                            // Reset this here

        if (final_stop)                                                 // If there is a final stop and no more jobs, bus is inactive, disable interrupts to prevent BTF
            I2C_ITConfig(I2Cx, I2C_IT_EVT | I2C_IT_ERR, DISABLE);       // Disable EVT and ERR interrupts while bus inactive
        busy = 0;
    }
}

///////////////////////////////////////////////////////////////////////////////
// I2C1 Error Interrupt Handler
///////////////////////////////////////////////////////////////////////////////

void I2C1_ER_IRQHandler(void)
{
    I2C_ER_Handler();
}

///////////////////////////////////////////////////////////////////////////////
// I2C1 Event Interrupt Handler
///////////////////////////////////////////////////////////////////////////////

void I2C1_EV_IRQHandler(void)
{
    I2C_EV_Handler();
}

///////////////////////////////////////////////////////////////////////////////
// I2C2 Error Interrupt Handler
///////////////////////////////////////////////////////////////////////////////

void I2C2_ER_IRQHandler(void)
{
    I2C_ER_Handler();
}

///////////////////////////////////////////////////////////////////////////////
// I2C2 Event Interrupt Handler
///////////////////////////////////////////////////////////////////////////////

void I2C2_EV_IRQHandler(void)
{
    I2C_EV_Handler();
}

///////////////////////////////////////////////////////////////////////////////
// I2C Write Buffer
///////////////////////////////////////////////////////////////////////////////

bool i2cWriteBuffer(I2C_TypeDef *I2C, uint8_t addr_, uint8_t reg_, uint8_t len_, uint8_t *data)
{
    uint8_t i;
    uint8_t my_data[16];
    uint32_t timeout = I2C_DEFAULT_TIMEOUT;

    I2Cx = I2C;

    addr = addr_ << 1;
    reg = reg_;
    writing = 1;
    reading = 0;
    write_p = my_data;
    read_p = my_data;
    bytes = len_;
    busy = 1;

    if (len_ > 16)
        return false;

    for (i = 0; i < len_; i++)
        my_data[i] = data[i];

    if (!(I2Cx->CR2 & I2C_IT_EVT))                                      // If we are restarting the driver
    {
        if (!(I2Cx->CR1 & I2C_CR1_START))                               // Ensure sending a start
        {
            while (I2Cx->CR1 & I2C_CR1_STOP) { ; }                      // Wait for any stop to finish sending
            I2C_GenerateSTART(I2Cx, ENABLE);                            // Send the start for the new job
        }
        I2C_ITConfig(I2Cx, I2C_IT_EVT | I2C_IT_ERR, ENABLE);            // Allow the interrupts to fire off again
    }

    while (busy && --timeout > 0);
    if (timeout == 0) {
        if (I2Cx == I2C1) i2c1ErrorCount++;
        if (I2Cx == I2C2) i2c2ErrorCount++;
        i2cInit(I2Cx);                                                  // Reinit peripheral + clock out garbage
        return false;
    }

    return true;
}

///////////////////////////////////////////////////////////////////////////////
// I2C Write
///////////////////////////////////////////////////////////////////////////////

bool i2cWrite(I2C_TypeDef *I2C, uint8_t addr_, uint8_t reg_, uint8_t data)
{
    return i2cWriteBuffer(I2C, addr_, reg_, 1, &data);
}

///////////////////////////////////////////////////////////////////////////////
// I2C Read
///////////////////////////////////////////////////////////////////////////////

bool i2cRead(I2C_TypeDef *I2C, uint8_t addr_, uint8_t reg_, uint8_t len, uint8_t *buf)
{
    uint32_t timeout = I2C_DEFAULT_TIMEOUT;

    I2Cx = I2C;

    addr = addr_ << 1;
    reg = reg_;
    writing = 0;
    reading = 1;
    read_p = buf;
    write_p = buf;
    bytes = len;
    busy = 1;

    if (!(I2Cx->CR2 & I2C_IT_EVT))                                      // If we are restarting the driver
    {
        if (!(I2Cx->CR1 & I2C_CR1_START))                               // Ensure sending a start
        {
            while (I2Cx->CR1 & I2C_CR1_STOP) { ; }                      // Wait for any stop to finish sending
            I2C_GenerateSTART(I2Cx, ENABLE);                            // Send the start for the new job
        }
        I2C_ITConfig(I2Cx, I2C_IT_EVT | I2C_IT_ERR, ENABLE);            // Allow the interrupts to fire off again
    }

    while (busy && --timeout > 0);
    if (timeout == 0) {
        if (I2Cx == I2C1) i2c1ErrorCount++;
        if (I2Cx == I2C2) i2c2ErrorCount++;
        i2cInit(I2Cx);                                                  // Reinit peripheral + clock out garbage
        return false;
    }

    return true;
}

///////////////////////////////////////////////////////////////////////////////
// I2C Unstick
///////////////////////////////////////////////////////////////////////////////

static void i2cUnstick(I2C_TypeDef *I2C)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    uint8_t i;

    GPIO_StructInit(&GPIO_InitStructure);

    ///////////////////////////////////

    if (I2C == I2C1)
    {
	    GPIO_InitStructure.GPIO_Pin   = I2C1_SCL_PIN | I2C1_SDA_PIN;
        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
      //GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;

        GPIO_Init(I2C1_GPIO, &GPIO_InitStructure);

        GPIO_SetBits(I2C1_GPIO, I2C1_SCL_PIN | I2C1_SDA_PIN);

        for (i = 0; i < 8; i++)
        {
            while (!GPIO_ReadInputDataBit(I2C1_GPIO, I2C1_SCL_PIN))  // Wait for any clock stretching to finish
                delayMicroseconds(3);                                // 2.5 would be 400 kHz, 3 is 333.33333 kHz

            // Pull low
            GPIO_ResetBits(I2C1_GPIO, I2C1_SCL_PIN);                 // Set bus low
            delayMicroseconds(3);                                    // 2.5 would be 400 kHz, 3 is 333.33333 kHz
            // Release high again
            GPIO_SetBits(I2C1_GPIO, I2C1_SCL_PIN);                   // Set bus high
            delayMicroseconds(3);                                    // 2.5 would be 400 kHz, 3 is 333.33333 kHz
        }

        // Generate a start then stop condition

        GPIO_ResetBits(I2C1_GPIO, I2C1_SDA_PIN);                     // Set bus data low
        delayMicroseconds(3);                                        // 2.5 would be 400 kHz, 3 is 333.33333 kHz
        GPIO_ResetBits(I2C1_GPIO, I2C1_SCL_PIN);                     // Set bus scl low
        delayMicroseconds(3);                                        // 2.5 would be 400 kHz, 3 is 333.33333 kHz
        GPIO_SetBits(I2C1_GPIO, I2C1_SCL_PIN);                       // Set bus scl high
        delayMicroseconds(3);                                        // 2.5 would be 400 kHz, 3 is 333.33333 kHz
        GPIO_SetBits(I2C1_GPIO, I2C1_SDA_PIN);                       // Set bus sda high
        delayMicroseconds(3);                                        // 2.5 would be 400 kHz, 3 is 333.33333 kHz
	}

	///////////////////////////////////

	if (I2C == I2C2)
	{
        GPIO_InitStructure.GPIO_Pin   = I2C2_SCL_PIN | I2C2_SDA_PIN;
        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
      //GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;

        GPIO_Init(I2C2_GPIO, &GPIO_InitStructure);

        GPIO_SetBits(I2C2_GPIO, I2C2_SCL_PIN | I2C2_SDA_PIN);

        for (i = 0; i < 8; i++)
        {
            while (!GPIO_ReadInputDataBit(I2C2_GPIO, I2C2_SCL_PIN))  // Wait for any clock stretching to finish
                delayMicroseconds(3);                                // 2.5 would be 400 kHz, 3 is 333.33333 kHz

            // Pull low
            GPIO_ResetBits(I2C2_GPIO, I2C2_SCL_PIN);                 // Set bus low
            delayMicroseconds(3);                                    // 2.5 would be 400 kHz, 3 is 333.33333 kHz
            // Release high again
            GPIO_SetBits(I2C2_GPIO, I2C2_SCL_PIN);                   // Set bus high
            delayMicroseconds(3);                                    // 2.5 would be 400 kHz, 3 is 333.33333 kHz
        }

        // Generate a start then stop condition

        GPIO_ResetBits(I2C2_GPIO, I2C2_SDA_PIN);                     // Set bus data low
        delayMicroseconds(3);                                        // 2.5 would be 400 kHz, 3 is 333.33333 kHz
        GPIO_ResetBits(I2C2_GPIO, I2C2_SCL_PIN);                     // Set bus scl low
        delayMicroseconds(3);                                        // 2.5 would be 400 kHz, 3 is 333.33333 kHz
        GPIO_SetBits(I2C2_GPIO, I2C2_SCL_PIN);                       // Set bus scl high
        delayMicroseconds(3);                                        // 2.5 would be 400 kHz, 3 is 333.33333 kHz
        GPIO_SetBits(I2C2_GPIO, I2C2_SDA_PIN);                       // Set bus sda high
        delayMicroseconds(3);                                        // 2.5 would be 400 kHz, 3 is 333.33333 kHz
    }

    ///////////////////////////////////
}

///////////////////////////////////////////////////////////////////////////////
// I2C Initialize
///////////////////////////////////////////////////////////////////////////////

void i2cInit(I2C_TypeDef *I2C)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    I2C_InitTypeDef  I2C_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    ///////////////////////////////////

    if (I2C == I2C1)
    {
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1,  ENABLE);

        i2cUnstick(I2C);                                         // Clock out stuff to make sure slaves arent stuck

        GPIO_StructInit(&GPIO_InitStructure);
        I2C_StructInit(&I2C_InitStructure);

        // Init pins
        GPIO_InitStructure.GPIO_Pin   = I2C1_SCL_PIN | I2C1_SDA_PIN;
        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
        //GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;

        GPIO_Init(I2C1_GPIO, &GPIO_InitStructure);

        GPIO_PinAFConfig(I2C1_GPIO, I2C1_SCL_PIN_SOURCE, GPIO_AF_I2C1);
        GPIO_PinAFConfig(I2C1_GPIO, I2C1_SDA_PIN_SOURCE, GPIO_AF_I2C1);

        // Init I2C
        I2C_ITConfig(I2C1, I2C_IT_EVT | I2C_IT_ERR, DISABLE);

        I2C_DeInit(I2C1);

        I2C_StructInit(&I2C_InitStructure);

        I2C_InitStructure.I2C_ClockSpeed          = 400000;
      //I2C_InitStructure.I2C_Mode                = I2C_Mode_I2C;
      //I2C_InitStructure.I2C_DutyCycle           = I2C_DutyCycle_2;
      //I2C_InitStructrue.I2C_OwnAddress1         = 0;
      //I2C_InitStructrue.I2C_Ack                 = I2C_Ack_Disable;
      //I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;

        I2C_Init(I2C1, &I2C_InitStructure);

        I2C_Cmd(I2C1, ENABLE);

        // I2C ER Interrupt
        NVIC_InitStructure.NVIC_IRQChannel                   = I2C1_ER_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
        NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;

        NVIC_Init(&NVIC_InitStructure);

        // I2C EV Interrupt
        NVIC_InitStructure.NVIC_IRQChannel                   = I2C1_EV_IRQn;
      //NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
      //NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
      //NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;

        NVIC_Init(&NVIC_InitStructure);
    }

    ///////////////////////////////////

    if (I2C == I2C2)
    {
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2,  ENABLE);

        i2cUnstick(I2C);                                         // Clock out stuff to make sure slaves arent stuck

        GPIO_StructInit(&GPIO_InitStructure);
        I2C_StructInit(&I2C_InitStructure);

        // Init pins
        GPIO_InitStructure.GPIO_Pin   = I2C2_SCL_PIN | I2C2_SDA_PIN;
        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
        //GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;

        GPIO_Init(I2C2_GPIO, &GPIO_InitStructure);

        GPIO_PinAFConfig(I2C2_GPIO, I2C2_SCL_PIN_SOURCE, GPIO_AF_I2C2);
        GPIO_PinAFConfig(I2C2_GPIO, I2C2_SDA_PIN_SOURCE, GPIO_AF_I2C2);

        // Init I2C
        I2C_ITConfig(I2C2, I2C_IT_EVT | I2C_IT_ERR, DISABLE);

        I2C_DeInit(I2C2);

        I2C_StructInit(&I2C_InitStructure);

        I2C_InitStructure.I2C_ClockSpeed          = 400000;
      //I2C_InitStructure.I2C_Mode                = I2C_Mode_I2C;
      //I2C_InitStructure.I2C_DutyCycle           = I2C_DutyCycle_2;
      //I2C_InitStructrue.I2C_OwnAddress1         = 0;
      //I2C_InitStructrue.I2C_Ack                 = I2C_Ack_Disable;
      //I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;

        I2C_Init(I2C2, &I2C_InitStructure);

        I2C_Cmd(I2C2, ENABLE);

        // I2C ER Interrupt
        NVIC_InitStructure.NVIC_IRQChannel                   = I2C2_ER_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
        NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;

          NVIC_Init(&NVIC_InitStructure);

        // I2C EV Interrupt
        NVIC_InitStructure.NVIC_IRQChannel                   = I2C2_EV_IRQn;
      //NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
      //NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
      //NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;

        NVIC_Init(&NVIC_InitStructure);
    }

    ///////////////////////////////////
}

///////////////////////////////////////////////////////////////////////////////
// Get I2C Error Count
///////////////////////////////////////////////////////////////////////////////

uint16_t i2cGetErrorCounter(I2C_TypeDef *I2C)
{
    if (I2C == I2C1)
    	return i2c1ErrorCount;
    else
    	return i2c2ErrorCount;
}

///////////////////////////////////////////////////////////////////////////////

