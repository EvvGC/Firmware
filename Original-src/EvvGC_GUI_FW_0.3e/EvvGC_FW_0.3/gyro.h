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

//struct gyro_data{
//	uint8_t ACCRead[6];
//	uint8_t GyroRead[6];
//	uint8_t data;
//};
//extern struct gyro_data Gyro;

extern float acc_pitch_angle, acc_roll_angle, accADC_x, accADC_y, accADC_z, gyroADC_x, gyroADC_x_last, gyroADC_y, gyroADC_z, acc_pitch_angle_vid, acc_roll_angle_vid;
//extern uint8_t ACCread[6], GYROread[6];
extern short int gyroADC_PITCH, gyroADC_ROLL, gyroADC_YAW, accADC_ROLL, accADC_PITCH, accADC_YAW, printcounter;
extern float gyroADC_ROLL_offset, gyroADC_PITCH_offset, gyroADC_YAW_offset, gyroADC_y_last;

void MPU6050_Init(void);
void MPU6050_Gyro_get(void);
void MPU6050_ACC_get(void);
void MPU6050_Gyro_calibration(void);

#endif /* GYRO_H_ */
