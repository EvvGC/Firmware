/*
 * 	engine.c
 *
 *	Created on: Jun 26, 2013
 *		Author: Denis aka caat
 */

#include "engine.h"
#include <math.h>
#include "adc.h"
#include "gyro.h"
#include "utils.h"
#include "config.h"
#include "pwm.h"
#include "rc.h"
#include "comio.h"
#include "systick.h"
#include "stopwatch.h"
#include "I2C.h"
#include "usb.h"

int   debugPrint = 0;
int   debugPerf  = 0;
int   debugCnt   = 0;

float pitch_angle_true;
float pitch, Gyro_Pitch_angle, pitch_setpoint = 0, pitch_Error_last, pitch_angle = 0, pitch_angle_correction;
float roll,  Gyro_Roll_angle,  roll_setpoint = 0,  roll_Error_last,  roll_angle = 0,  roll_angle_correction;
float yaw,   Gyro_Yaw_angle,   yaw_setpoint = 0,   yaw_Error_last,   yaw_angle = 0,   yaw_angle_correction;
float sinusas[91] =
{
    0.000, 0.017, 0.035, 0.052, 0.070, 0.087, 0.105, 0.122, 0.139, 0.156,
    0.174, 0.191, 0.208, 0.225, 0.242, 0.259, 0.276, 0.292, 0.309, 0.326,
    0.342, 0.358, 0.375, 0.391, 0.407, 0.423, 0.438, 0.454, 0.469, 0.485,
    0.500, 0.515, 0.530, 0.545, 0.559, 0.574, 0.588, 0.602, 0.616, 0.629,
    0.643, 0.656, 0.669, 0.682, 0.695, 0.707, 0.719, 0.731, 0.743, 0.755,
    0.766, 0.777, 0.788, 0.799, 0.809, 0.819, 0.829, 0.839, 0.848, 0.857,
    0.866, 0.875, 0.883, 0.891, 0.899, 0.906, 0.914, 0.920, 0.927, 0.934,
    0.940, 0.945, 0.951, 0.956, 0.961, 0.966, 0.970, 0.974, 0.978, 0.982,
    0.985, 0.988, 0.990, 0.993, 0.995, 0.996, 0.998, 0.999, 0.999, 1.000,
    1.000
};

float ADC1Ch1_vid, ADC1Ch13_vid, rc4_avg, rotation;
static int printcounter;

void roll_PID(void)
{
    float Error_current = roll_setpoint + roll_angle * 1000;
    float P = Error_current * (float)configData[1] / 100;
    float D = (float)configData[4] / 100 * (Error_current - roll_Error_last);

    roll_Error_last = Error_current;

	SetRollMotor(P + D, configData[7]);
}

void pitch_PID(void)
{
    float Error_current = pitch_setpoint + pitch_angle * 1000;
    float P = Error_current * (float)configData[0] / 100;
    float D = (float)configData[3] / 100 * (Error_current - pitch_Error_last);

    pitch_Error_last = Error_current;
		
	SetPitchMotor(P + D, configData[6]);
}

void yaw_PID(void)
{
    float Error_current = yaw_setpoint + yaw_angle * 1000;
    float P = Error_current * (float)configData[2] / 100;
    float D = (float)configData[5] / 100 * (Error_current - yaw_Error_last);

    yaw_Error_last = Error_current;

	SetYawMotor(P + D, configData[8]);
}


float constrain(float value, float low, float high)
{
	if(value < low)  return low;
	if(value > high) return high;
	return value;
}

void engineProcess(float dt)
{
	static int loopCounter;
	static float acc_pitch_angle_vid = 0.0;
	static float acc_roll_angle_vid  = 0.0;
	tStopWatch sw;
	
	loopCounter++;
    LEDon();
    DEBUG_LEDoff();

	StopWatchInit(&sw);
    MPU6050_ACC_get(); //Getting Accelerometer data
	unsigned long tAccGet = StopWatchLap(&sw);

	int aux3 = GetAUX3();
	int aux4 = GetAUX4();
	int rc4Deg =  aux4 * 90 / 1000;
	
    float acc_roll_angle  = -(atan2f(accADC_x, accADC_z)) + (configData[11] - 50.00) * 0.0035; //Calculating pitch ACC angle+callibration
    float acc_pitch_angle = +(atan2f(accADC_y, accADC_z));   //Calculating roll ACC angle
	unsigned long tAccAngle = StopWatchLap(&sw);;

    MPU6050_Gyro_get(); //Getting Gyroscope data
	unsigned long tGyroGet = StopWatchLap(&sw);;

    acc_roll_angle_vid  = ((acc_roll_angle_vid * 99.00)  + acc_roll_angle)  / 100.00; //Averaging pitch ACC values
    acc_pitch_angle_vid = ((acc_pitch_angle_vid * 99.00) + acc_pitch_angle) / 100.00; //Averaging roll  ACC values

    float sinus   = sinusas[rc4Deg];      //Calculating sinus
    float cosinus = sinusas[90 - rc4Deg]; //Calculating cosinus

    float roll = - gyroADC_z * sinus + gyroADC_y * cosinus;
    roll_angle = (roll_angle + roll * dt)    + 0.0002 * (acc_roll_angle_vid - roll_angle); //Roll Horizon
    roll_angle_correction = constrain(roll_angle * 50.0, -1.0, 1.0);
    roll_setpoint += roll_angle_correction; //Roll return to zero after collision

	
    yaw_angle = (yaw_angle + gyroADC_z * dt); //Yaw

	
    pitch_angle_true = ((pitch_angle_true  + gyroADC_x * dt) + 0.0002 * (acc_pitch_angle_vid - pitch_angle_true)); //Pitch Horizon
    rotation += gyroADC_x * dt;
    ADC1Ch1_vid = ((ADC1Ch1_vid * 99.00) + (readADC1(1) / 4000.00)) / 100.00;	//Averaging ADC values
    ADC1Ch1_vid = 0.00;

    rc4_avg = ((rc4_avg * 499.00) + rc4Deg) / 500.00;	//Averaging RC4 values
    pitch_angle = pitch_angle_true - rc4_avg; //Adding angle

    pitch_angle_correction = constrain(pitch_angle * 50.0, -1.0, 1.0);
    pitch_setpoint += pitch_angle_correction; //Pitch return to zero after collision



    ADC1Ch13_vid = ((ADC1Ch13_vid * 99.00) + ((float)(readADC1(13) - 2000) / 4000.00)) / 100.00;	//Averaging ADC values

    if (configData[10] == '0') //Yaw AutoPan
    {
        yaw_angle = (yaw_angle + gyroADC_z * dt) + 0.01 * (ADC1Ch13_vid - yaw_angle);
    } else if (configData[10] == '1') { //Yaw RCPan
        yaw_angle = (yaw_angle + gyroADC_z * dt);
    } 

    yaw_angle_correction = constrain(yaw_angle * 50.0, -1.0, 1.0);
    yaw_setpoint = yaw_setpoint + yaw_angle_correction; //Yaw return to zero after collision

	unsigned long tCalc = StopWatchLap(&sw);
	
    pitch_PID();
    roll_PID();
    yaw_PID();

	unsigned long tPID = StopWatchLap(&sw);
	unsigned long tAll = StopWatchTotal(&sw);

    printcounter++;
    //if (printcounter >= 500 || dt > 0.0021) {		
    if (printcounter >= 500) {		
		if(debugPrint) {
			print("loop %6d: dt %f, aux3 %4d, aux4 %4d, I2Cerr %4d, angles: roll %6.1f, pitch %6.1f, yaw %6.1f\r\n", 
				  loopCounter, dt, aux3, aux4, I2Cerrorcount, Rad2Deg(roll_angle), Rad2Deg(pitch_angle), Rad2Deg(yaw_angle));
		}
		
		if(debugPerf) {
			extern float GetIdlePerf(void);
			print("idle %5.2f%%, time[µs]: engine %4d, IMU acc %4d, gyro %4d, angle %4d, calc %4d, PID %4d\r\n", 
				GetIdlePerf(), tAll, tAccGet, tGyroGet, tAccAngle, tCalc, tPID);
		}
        
		if(debugCnt) {
			print("Counter min %3d, %3d, %3d,  max %4d, %4d, %4d, count %3d, %3d, %3d, usbOverrun %4d\r\n", 
				MinCnt[ROLL], MinCnt[PITCH], MinCnt[YAW], 
				MaxCnt[ROLL], MaxCnt[PITCH], MaxCnt[YAW], 
				IrqCnt[ROLL], IrqCnt[PITCH], IrqCnt[YAW],
				usbOverrun()); 
		}
		//MaxCntClear();
		printcounter = 0;
    }

    LEDoff();
}

