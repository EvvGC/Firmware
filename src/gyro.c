/*
 *  gyro.c
 *
 *  Created on: Jun 26, 2013
 *      Author: Denis aka caat
 */
#include <stdint.h>
#include <math.h>
#include "gyro.h"
#include "i2c.h"
#include "utils.h"
#include "pins.h"
#include "pwm.h"
#include "definitions.h"
#include "engine.h"

static float gyroADC_ROLL_offset, gyroADC_PITCH_offset, gyroADC_YAW_offset;
static short int gyroADC_PITCH, gyroADC_ROLL, gyroADC_YAW;

int MPU6050_Init(void)
{
    uint8_t mpu_adr;

    // Check to make sure there is a device out there and its on the
    // correct address
    I2C1_Start();
    I2C1_SendByte((0xD1 & 0xFE));//fe-0(Write)
    I2C1_WaitAck();
    I2C1_SendByte(0x75); // Who Am I
    I2C1_WaitAck();
    I2C1_Stop();

    Delay_ms(1);

    I2C1_Start();
    I2C1_SendByte((0xD1 & 0xFF));//ff-1(Read)
    I2C1_WaitAck();

    mpu_adr = I2C1_ReceiveByte();//receive

    I2C1_NoAck();
    I2C1_Stop();

    // if wrong address or no device then bail out with an error
    if (mpu_adr != 0x68)
    {
        return -1;
    }

    Delay_ms(5);

    // force a device reset
    I2C1_Start();
    I2C1_SendByte((0xD1 & 0xFE));//fe-0(Write)
    I2C1_WaitAck();
    I2C1_SendByte(0x6B); // Force a reset
    I2C1_WaitAck();
    I2C1_SendByte(0x80);
    I2C1_WaitAck();
    I2C1_Stop();

    Delay_ms(150);

    // set the internal clock to be the Z AXIS gyro
    I2C1_Start();
    I2C1_SendByte((0xD1 & 0xFE));//fe-0(Write)
    I2C1_WaitAck();
    I2C1_SendByte(0x6B);
    I2C1_WaitAck();
    I2C1_SendByte(0x03); // clock source AKA - changed from 0x00 (internal clock)
    I2C1_WaitAck();
    I2C1_Stop();

    Delay_ms(5);

    //  turn off all sleep modes
    I2C1_Start();
    I2C1_SendByte((0xD1 & 0xFE));//fe-0(Write)
    I2C1_WaitAck();
    I2C1_SendByte(0x6C);
    I2C1_WaitAck();
    I2C1_SendByte(0x00); // wake up ctrl
    I2C1_WaitAck();
    I2C1_Stop();

    Delay_ms(5);

    //  Set the sample rate on the accel and refresh rate on the gyro
    I2C1_Start();
    I2C1_SendByte((0xD1 & 0xFE));//fe-0(Write)
    I2C1_WaitAck();
    I2C1_SendByte(0x19); // Sample output rate
    I2C1_WaitAck();
    I2C1_SendByte(0x00);
    I2C1_WaitAck();
    I2C1_Stop();

    Delay_ms(5);

    // turn on the built in LPF
    I2C1_Start();
    I2C1_SendByte((0xD1 & 0xFE));//fe-0(Write)
    I2C1_WaitAck();
    I2C1_SendByte(0x1A);
    I2C1_WaitAck();
    I2C1_SendByte(0x00);    //low pass disable AKA - was 0x02 for 98hz
    I2C1_WaitAck();
    I2C1_Stop();

    Delay_ms(5);

    // set the gyro scale
    I2C1_Start();
    I2C1_SendByte((0xD1 & 0xFE));//fe-0(Write)
    I2C1_WaitAck();
    I2C1_SendByte(0x1B);
    I2C1_WaitAck();
    I2C1_SendByte(0x00); //set to 250LSB/Deg/s
    I2C1_WaitAck();
    I2C1_Stop();

    Delay_ms(5);

    // set the accel scale
    I2C1_Start();
    I2C1_SendByte((0xD1 & 0xFE));//fe-0(Write)
    I2C1_WaitAck();
    I2C1_SendByte(0x1C);
    I2C1_WaitAck();
    I2C1_SendByte(0x00); //set to accel to +/-2g scale AKA - was 0x08 for +/-4g
    I2C1_WaitAck();
    I2C1_Stop();

    Delay_ms(5);

    //  configure the interrupt(s) pin because we don't use it
    I2C1_Start();
    I2C1_SendByte((0xD1 & 0xFE));//fe-0(Write)
    I2C1_WaitAck();
    I2C1_SendByte(0x37); // init pin config
    I2C1_WaitAck();
    I2C1_SendByte(0x00);
    I2C1_WaitAck();
    I2C1_Stop();

    Delay_ms(5);

    // disable the interrupt pin(s)
    I2C1_Start();
    I2C1_SendByte((0xD1 & 0xFE));//fe-0(Write)
    I2C1_WaitAck();
    I2C1_SendByte(0x38); // init enable
    I2C1_WaitAck();
    I2C1_SendByte(0x00);
    I2C1_WaitAck();
    I2C1_Stop();

    Delay_ms(5);

/*
    // this was bad code and was removed
    I2C1_Start();
    I2C1_SendByte((0xD1 & 0xFE));//fe-0(Write)
    I2C1_WaitAck();
    I2C1_SendByte(0x6A);
    I2C1_WaitAck();
    I2C1_SendByte(0x01); // reset signal paths
    I2C1_WaitAck();
    I2C1_Stop();

    Delay_ms(5);
*/

    return 0;
}

void MPU6050_get(int cmd, uint8_t read[6])
{
    I2Cerror = 0;

    I2C1_Start();
    I2C1_SendByte((0xD1 & 0xFE));//fe-0(Write)
    I2C1_WaitAck();

    if (I2Cerror == 0)
    {
        I2C1_SendByte(cmd);
        I2C1_WaitAck();

        if (I2Cerror == 0)
        {
            I2C1_Stop();
            I2C1_Start();
            I2C1_SendByte((0xD1 & 0xFF));//ff-1(Read)
            I2C1_WaitAck();

            if (I2Cerror == 0)
            {
                read[0] = I2C1_ReceiveByte(); //receive
                I2C1_Ack();
                read[1] = I2C1_ReceiveByte(); //receive
                I2C1_Ack();
                read[2] = I2C1_ReceiveByte(); //receive
                I2C1_Ack();
                read[3] = I2C1_ReceiveByte(); //receive
                I2C1_Ack();
                read[4] = I2C1_ReceiveByte(); //receive
                I2C1_Ack();
                read[5] = I2C1_ReceiveByte(); //receive
                I2C1_NoAck();
                I2C1_Stop();
            }
        }
    }
}

void MPU6050_ACC_get(float *AccData)
{
    uint8_t read[6];

    MPU6050_get(0x3B, read);

    if (I2Cerror == 0)
    {
        AccData[X_AXIS] = (short)((read[0] << 8) | read[1]);
        AccData[Y_AXIS] = (short)((read[2] << 8) | read[3]);
        AccData[Z_AXIS] = (short)((read[4] << 8) | read[5]);
    }
}

void MPU6050_Gyro_get(float *GyroData)
{
    uint8_t read[6];

    MPU6050_get(0x43, read);

    if (I2Cerror == 0)
    {
        float gyroScaleFactor = 7505.747116f;// 8000.0f;//     2.0F/131.0F * M_PI/180.0F;

        gyroADC_ROLL  = (short)((read[0] << 8) | read[1]);
        GyroData[X_AXIS] = ((float)gyroADC_ROLL - gyroADC_ROLL_offset) / gyroScaleFactor;
        // GyroData[X_AXIS] = ((float)gyroADC_ROLL  - gyroADC_ROLL_offset)  * gyroScaleFactor;

        gyroADC_PITCH = (short)((read[2] << 8) | read[3]);
        GyroData[Y_AXIS] = ((float)gyroADC_PITCH - gyroADC_PITCH_offset) / gyroScaleFactor;
        // GyroData[Y_AXIS] = ((float)gyroADC_PITCH - gyroADC_PITCH_offset) * gyroScaleFactor;

        gyroADC_YAW   = (short)((read[4] << 8) | read[5]);
        GyroData[Z_AXIS] = ((float)gyroADC_YAW - gyroADC_YAW_offset) / gyroScaleFactor;
        // GyroData[Z_AXIS] = ((float)gyroADC_YAW   - gyroADC_YAW_offset)   * gyroScaleFactor;
    }
}

void MPU6050_Gyro_calibration(void)
{
    uint8_t i;
    int loops = 100;
    float InitGyroData[3];

    for (i = 0; i < loops; i++)
    {
        MPU6050_Gyro_get(InitGyroData);

        gyroADC_ROLL_offset  += gyroADC_ROLL;
        gyroADC_PITCH_offset += gyroADC_PITCH;
        gyroADC_YAW_offset   += gyroADC_YAW;
        Delay_ms(2);
    }

    gyroADC_ROLL_offset  /= loops;
    gyroADC_PITCH_offset /= loops;
    gyroADC_YAW_offset   /= loops;

    Delay_ms(5);
}
