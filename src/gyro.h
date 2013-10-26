/*
 *  gyro.h
 *
 *  Created on: Jun 26, 2013
 *      Author: Denis aka caat
 *
 * MPU6050 gyro via i2c bus.
 */

#ifndef GYRO_H_
#define GYRO_H_

#define MPU6050_I2C I2C2 //MPU6050 Bus

int MPU6050_Init(void);
void MPU6050_Gyro_get(float *GyroData);
void MPU6050_ACC_get(float *AccData);
void MPU6050_Gyro_calibration(void);

#endif /* GYRO_H_ */
