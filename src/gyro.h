/*
 * 	gyro.h
 *
 *	Created on: Jun 26, 2013
 *		Author: Denis aka caat
 *
 * MPU6050 gyro via i2c bus.
 */

#ifndef GYRO_H_
#define GYRO_H_
#include <stdint.h>

#define MPU6050_I2C I2C2 //MPU6050 Bus

extern float accADC_x, accADC_y, accADC_z, gyroADC_x, gyroADC_x_last, gyroADC_y, gyroADC_z;
extern short int gyroADC_PITCH, gyroADC_ROLL, gyroADC_YAW, accADC_ROLL, accADC_PITCH, accADC_YAW;

int MPU6050_Init(void);
void MPU6050_Gyro_get(void);
void MPU6050_ACC_get(void);
void MPU6050_Gyro_calibration(void);

#endif /* GYRO_H_ */
