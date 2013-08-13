/*
 * 	gyro.c
 *
 *	Created on: Jun 26, 2013
 *		Author: Denis aka caat
 */
#include <math.h>
#include "gyro.h"
#include "i2c.h"
#include "utils.h"
#include "pins.h"

uint8_t ACCread[6], GYROread[6];
float gyroADC_ROLL_offset, gyroADC_PITCH_offset, gyroADC_YAW_offset;
float accADC_x, accADC_y, accADC_z, gyroADC_x, gyroADC_y, gyroADC_z;
short int gyroADC_PITCH, gyroADC_ROLL, gyroADC_YAW, accADC_ROLL, accADC_PITCH, accADC_YAW;

int MPU6050_Init(void)
{
    uint8_t mpu_adr;

    Delay_ms(1);

    I2C1_Start();
    I2C1_SendByte((0xD1 & 0xFE));//fe-0(Write)
    I2C1_WaitAck();
    I2C1_SendByte(0x75);
    I2C1_WaitAck();
    I2C1_Stop();

    Delay_ms(1);

    I2C1_Start();
    I2C1_SendByte((0xD1 & 0xFF));//ff-1(Read)
    I2C1_WaitAck();
    mpu_adr = I2C1_ReceiveByte();//receive
    I2C1_NoAck();
    I2C1_Stop();

    if (mpu_adr != 0x68)
    {
		return -1;
    }

    Delay_ms(5);

    I2C1_Start();
    I2C1_SendByte((0xD1 & 0xFE));//fe-0(Write)
    I2C1_WaitAck();
    I2C1_SendByte(0x19);
    I2C1_WaitAck();
    I2C1_SendByte(0x00);
    I2C1_WaitAck();
    I2C1_Stop();

    Delay_ms(5);

    I2C1_Start();
    I2C1_SendByte((0xD1 & 0xFE));//fe-0(Write)
    I2C1_WaitAck();
    I2C1_SendByte(0x1A);
    I2C1_WaitAck();
    I2C1_SendByte(0x02);//low pass
    I2C1_WaitAck();
    I2C1_Stop();

    Delay_ms(5);

    I2C1_Start();
    I2C1_SendByte((0xD1 & 0xFE));//fe-0(Write)
    I2C1_WaitAck();
    I2C1_SendByte(0x1B);
    I2C1_WaitAck();
    I2C1_SendByte(0x08);
    I2C1_WaitAck();
    I2C1_Stop();

    Delay_ms(5);

    I2C1_Start();
    I2C1_SendByte((0xD1 & 0xFE));//fe-0(Write)
    I2C1_WaitAck();
    I2C1_SendByte(0x37);
    I2C1_WaitAck();
    I2C1_SendByte(0x00);
    I2C1_WaitAck();
    I2C1_Stop();

    Delay_ms(5);

    I2C1_Start();
    I2C1_SendByte((0xD1 & 0xFE));//fe-0(Write)
    I2C1_WaitAck();
    I2C1_SendByte(0x38);
    I2C1_WaitAck();
    I2C1_SendByte(0x00);
    I2C1_WaitAck();
    I2C1_Stop();

    Delay_ms(5);

    I2C1_Start();
    I2C1_SendByte((0xD1 & 0xFE));//fe-0(Write)
    I2C1_WaitAck();
    I2C1_SendByte(0x6B);
    I2C1_WaitAck();
    I2C1_SendByte(0x00);
    I2C1_WaitAck();
    I2C1_Stop();

    Delay_ms(5);

    I2C1_Start();
    I2C1_SendByte((0xD1 & 0xFE));//fe-0(Write)
    I2C1_WaitAck();
    I2C1_SendByte(0x6A);
    I2C1_WaitAck();
    I2C1_SendByte(0x09);
    I2C1_WaitAck();
    I2C1_Stop();

    Delay_ms(5);
	
	return 0;
}

void MPU6050_ACC_get(void)
{
    I2Cerror = 0;

    I2C1_Start();
    I2C1_SendByte((0xD1 & 0xFE));//fe-0(Write)
    I2C1_WaitAck();

    if (I2Cerror == 0)
    {
        I2C1_SendByte(0x3B);
        I2C1_WaitAck();

        if (I2Cerror == 0)
        {
            I2C1_Stop();
            I2C1_Start();
            I2C1_SendByte((0xD1 & 0xFF));//ff-1(Read)
            I2C1_WaitAck();

            if (I2Cerror == 0)
            {
                ACCread[0] = I2C1_ReceiveByte(); //receive
                I2C1_Ack();
                ACCread[1] = I2C1_ReceiveByte(); //receive
                I2C1_Ack();
                ACCread[2] = I2C1_ReceiveByte(); //receive
                I2C1_Ack();
                ACCread[3] = I2C1_ReceiveByte(); //receive
                I2C1_Ack();
                ACCread[4] = I2C1_ReceiveByte(); //receive
                I2C1_Ack();
                ACCread[5] = I2C1_ReceiveByte(); //receive
                I2C1_NoAck();
                I2C1_Stop();
            }
        }
    }

    if (I2Cerror == 0)
    {
        accADC_ROLL  = (((ACCread[0] << 8) | ACCread[1]));
        accADC_x  = (accADC_ROLL);
        accADC_PITCH = (((ACCread[2] << 8) | ACCread[3]));
        accADC_y = (accADC_PITCH);
        accADC_YAW   = (((ACCread[4] << 8) | ACCread[5]));
        accADC_z    = (accADC_YAW);
    }
}

void MPU6050_Gyro_get(void)
{
    I2Cerror = 0;

    I2C1_Start();
    I2C1_SendByte((0xD1 & 0xFE));//fe-0(Write)
    I2C_delay();	//????
    I2C1_WaitAck();
    I2C_delay();

    if (I2Cerror == 0)
    {
        I2C1_SendByte(0x43);
        I2C_delay();
        I2C1_WaitAck();

        if (I2Cerror == 0)
        {
            I2C1_Stop();


            I2C1_Start();
            I2C1_SendByte((0xD1 & 0xFF));//ff-1(Read)
            I2C1_WaitAck();
            I2C_delay();

            if (I2Cerror == 0)
            {
                GYROread[0] = I2C1_ReceiveByte(); //receive
                I2C1_Ack();
                I2C_delay();
                GYROread[1] = I2C1_ReceiveByte(); //receive
                I2C1_Ack();
                I2C_delay();
                GYROread[2] = I2C1_ReceiveByte(); //receive
                I2C1_Ack();
                I2C_delay();
                GYROread[3] = I2C1_ReceiveByte(); //receive
                I2C1_Ack();
                I2C_delay();
                GYROread[4] = I2C1_ReceiveByte(); //receive
                I2C1_Ack();
                I2C_delay();
                GYROread[5] = I2C1_ReceiveByte(); //receive
                I2C1_NoAck();
            }
        }
    }

    I2C1_Stop();

    if (I2Cerror == 0)
    {
		float gyroScaleFactor = 131.0F/2.0F * 180.0F/M_PI;
        gyroADC_ROLL  = (((GYROread[0] << 8) | GYROread[1]));
        gyroADC_x = ((float)gyroADC_ROLL - gyroADC_ROLL_offset) / gyroScaleFactor;

        gyroADC_PITCH = (((GYROread[2] << 8) | GYROread[3]));
        gyroADC_y = ((float)gyroADC_PITCH - gyroADC_PITCH_offset) / gyroScaleFactor;

        gyroADC_YAW   = (((GYROread[4] << 8) | GYROread[5]));
        gyroADC_z = ((float)gyroADC_YAW - gyroADC_YAW_offset) / gyroScaleFactor;
    }
}

void MPU6050_Gyro_calibration(void)
{
    uint8_t i;
	int loops = 100;
    for (i = 0; i < loops; i++)
    {
        MPU6050_Gyro_get();

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
