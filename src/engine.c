/*
 *  engine.c
 *
 *  Created on: Jun 26, 2013
 *      Author: Denis aka caat
 */
#include <stdint.h>
#include <math.h>
#include "engine.h"
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
#include "definitions.h"
#include "usb.h"
#include "main.h"

int   debugPrint = 0;
int   debugPerf = 0;
int   debugSense = 0;
int   debugCnt = 0;
int   debugRC = 0;
int   debugOrient = 0;

float pitch, Gyro_Pitch_angle, pitch_setpoint = 0.0, pitch_Error_last,  pitch_angle_correction;
float roll,  Gyro_Roll_angle,  roll_setpoint = 0.0,  roll_Error_last,    roll_angle_correction;
float yaw,   Gyro_Yaw_angle,   yaw_setpoint = 0.0,   yaw_Error_last,      yaw_angle_correction;

float ADC1Ch13_yaw;

static float rollRCOffset = 0.0, pitchRCOffset = 0.0, yawRCOffset = 0.0;

static int printcounter;

float CameraOrient[EULAR];
float AccAngleSmooth[EULAR];

float AccData[NUMAXIS] = {0.0, 0.0, 0.0};
float GyroData[NUMAXIS] = {0.0, 0.0, 0.0};

float Step[NUMAXIS] = {0.0, 0.0, 0.0};
float RCSmooth[NUMAXIS] = {0.0, 0.0, 0.0};

void roll_PID(void)
{
    float Error_current = roll_setpoint + CameraOrient[ROLL] * 1000;
    float KP = Error_current * (float)configData[1] / 1000;
    float KD = (float)configData[4] / 100 * (Error_current - roll_Error_last);

    roll_Error_last = Error_current;

    SetRollMotor(KP + KD, configData[7]);
}

void pitch_PID(void)
{
    float Error_current = pitch_setpoint + CameraOrient[PITCH] * 1000;
    float KP = Error_current * (float)configData[0] / 1000;
    float KD = (float)configData[3] / 100 * (Error_current - pitch_Error_last);

    pitch_Error_last = Error_current;

    //print("error:%3.2f\n\r",Error_current);

    SetPitchMotor(KP + KD, configData[6]);
}

void yaw_PID(void)
{
    float Error_current = yaw_setpoint + CameraOrient[YAW] * 1000;
    float KP = Error_current * (float)configData[2] / 1000;
    float KD = (float)configData[5] / 100 * (Error_current - yaw_Error_last);

    yaw_Error_last = Error_current;

    SetYawMotor(KP + KD, configData[8]);
}

float constrain(float value, float low, float high)
{
    if (value < low)
        return low;

    if (value > high)
        return high;

    return value;
}

/*
  Limits the Pitch angle
*/
float Limit_Pitch(float step, float pitch)
{
    if (pitch < PITCH_UP_LIMIT && step > 0)
    {
        step = 0.0;
    }

    if (pitch > PITCH_DOWN_LIMIT && step < 0)
    {
        step = 0.0;
    }

    return step;
}

void Init_Orientation()
{

    int init_loops = 150;
    float AccAngle[NUMAXIS];
    int i;

    for (i = 0; i <= init_loops; i++)
    {
        MPU6050_ACC_get(AccData); //Getting Accelerometer data
        AccAngle[ROLL]  = -(atan2f(AccData[X_AXIS], AccData[Z_AXIS]));   //Calculating pitch ACC angle+callibration
        AccAngle[PITCH] = +(atan2f(AccData[Y_AXIS], AccData[Z_AXIS]));   //Calculating roll ACC angle

        AccAngleSmooth[ROLL]  = ((AccAngleSmooth[ROLL] * 99.00)  + AccAngle[ROLL])  / 100.00; //Averaging pitch ACC values
        AccAngleSmooth[PITCH] = ((AccAngleSmooth[PITCH] * 99.00) + AccAngle[PITCH]) / 100.00; //Averaging roll  ACC values
        Delay_us(5);
    }

    CameraOrient[PITCH] = AccAngleSmooth[PITCH];
    CameraOrient[ROLL] = AccAngleSmooth[ROLL];
    CameraOrient[YAW] = 0;
}

void Get_Orientation(float *SmoothAcc, float *Orient, float *AccData, float *GyroData, float dt)
{
    float AccAngle[EULAR];
    float GyroRate[EULAR];

    AccAngle[ROLL]  = -(atan2f(AccData[X_AXIS], AccData[Z_AXIS]));   //Calculating pitch ACC angle+callibration
    AccAngle[PITCH] = +(atan2f(AccData[Y_AXIS], AccData[Z_AXIS]));   //Calculating roll ACC angle
    SmoothAcc[ROLL]  = ((SmoothAcc[ROLL] * 99.00)  + AccAngle[ROLL])  / 100.00; //Averaging pitch ACC values
    SmoothAcc[PITCH] = ((SmoothAcc[PITCH] * 99.00) + AccAngle[PITCH]) / 100.00; //Averaging roll  ACC values

    GyroRate[PITCH] =  GyroData[X_AXIS];
    Orient[PITCH] = ((Orient[PITCH]  + GyroRate[PITCH] * dt) + 0.0002 * (SmoothAcc[PITCH] - Orient[PITCH])); //Pitch Horizon

    GyroRate[ROLL] = -GyroData[Z_AXIS] * sinf(((Orient[PITCH]))) + GyroData[Y_AXIS] * cosf(fabs(Orient[PITCH]));
    Orient[ROLL] = (Orient[ROLL] + GyroRate[ROLL] * dt)    + 0.0002 * (SmoothAcc[ROLL] - Orient[ROLL]); //Roll Horizon

    GyroRate[YAW]  = -GyroData[Z_AXIS] * cosf((fabsf(Orient[PITCH]))) - GyroData[Y_AXIS] * sinf(((Orient[PITCH]))); //presuming Roll is horizontal
    Orient[YAW] = (Orient[YAW] + GyroRate[YAW] * dt); //Yaw
    //print("%3.3f\r\n",Orient[PITCH]);
}

void engineProcess(float dt)
{
    static int loopCounter;
    tStopWatch sw;

    loopCounter++;
    LEDon();
    DEBUG_LEDoff();

    StopWatchInit(&sw);
    MPU6050_ACC_get(AccData); // Getting Accelerometer data
    unsigned long tAccGet = StopWatchLap(&sw);

    MPU6050_Gyro_get(GyroData); // Getting Gyroscope data
    unsigned long tGyroGet = StopWatchLap(&sw);

    Get_Orientation(AccAngleSmooth, CameraOrient, AccData, GyroData, dt);
    unsigned long tAccAngle = StopWatchLap(&sw);

    // if we enable RC control
    if (configData[9] == '1')
    {
        // Get the RX values and Averages
        Get_RC_Step(Step, RCSmooth); // Get RC movement on all three AXIS
        Step[PITCH] = Limit_Pitch(Step[PITCH], CameraOrient[PITCH]); // limit pitch to defined limits in header
    }

    // Pitch adjustments
    pitch_setpoint += Step[PITCH];
    pitchRCOffset += Step[PITCH] / 1000;

    pitch_angle_correction = constrain((CameraOrient[PITCH] + pitchRCOffset) * 50.0, -1.0, 1.0);
    pitch_setpoint += pitch_angle_correction; // Pitch return to zero after collision

    // Roll Adjustments
    roll_setpoint += Step[ROLL];
    rollRCOffset += Step[ROLL] / 1000;

    roll_angle_correction = constrain((CameraOrient[ROLL] + rollRCOffset + (((configData[11] - 100) / 10.0) / R2D)) * 50.0, -1.0, 1.0);
    roll_setpoint += roll_angle_correction; //Roll return to zero after collision

    // if we enabled AutoPan on Yaw
    if (configData[10] == '0')
    {
        ADC1Ch13_yaw = ((ADC1Ch13_yaw * 99.0) + ((float)(readADC1(13) - 2000) / 4000.0)) / 100.0;  // Average ADC value
    }

    // Yaw Adjustments
    yaw_setpoint += Step[YAW];
    yawRCOffset += Step[YAW] / 1000;

    // if AutoPan is enabled
    if (configData[10] == '0')
    {
        CameraOrient[YAW] = CameraOrient[YAW] + 0.01 * (ADC1Ch13_yaw - CameraOrient[YAW]);
    }

    yaw_angle_correction = constrain((CameraOrient[YAW] + yawRCOffset) * 50.0, -1.0, 1.0);
    yaw_setpoint += yaw_angle_correction; // Yaw return to zero after collision

    unsigned long tCalc = StopWatchLap(&sw);

    pitch_PID();
    roll_PID();
    yaw_PID();

    unsigned long tPID = StopWatchLap(&sw);
    unsigned long tAll = StopWatchTotal(&sw);

    printcounter++;

    //if (printcounter >= 500 || dt > 0.0021)
    if (printcounter >= 500)
    {
        if (debugPrint)
        {
            print("Loop: %7d, I2CErrors: %d, angles: roll %7.2f, pitch %7.2f, yaw %7.2f, rollSP: %3.2f, pitchSP: %3.2f\r\n",
                  loopCounter, I2Cerrorcount, Rad2Deg(CameraOrient[ROLL]), Rad2Deg(CameraOrient[PITCH]), Rad2Deg(CameraOrient[YAW]),
                  Rad2Deg(roll_setpoint) / 1000, Rad2Deg(pitch_setpoint) / 1000);
        }

        if (debugSense)
        {
            print(" dt %f, AccData: %6.0f | %6.0f | %6.0f, GyroData %7.3f | %7.3f | %7.3f \r\n",
                  dt, AccData[X_AXIS], AccData[Y_AXIS], AccData[Z_AXIS], GyroData[X_AXIS], GyroData[Y_AXIS], GyroData[Z_AXIS]);
        }

        if (debugPerf)
        {
            print("idle: %5.2f%%, time[µs]: attitude est. %4d, IMU acc %4d, gyro %4d, angle %4d, calc %4d, PID %4d\r\n",
                  GetIdlePerf(), tAll, tAccGet, tGyroGet, tAccAngle, tCalc, tPID);
        }

        if (debugRC)
        {
            print(" RC4avg: %7.2f |  RC2avg: %7.2f |  RC3avg: %7.2f | PStep:%7.3f  YStep: %7.3f  RStep: %7.3f\r\n",
                  RCSmooth[PITCH], RCSmooth[ROLL], RCSmooth[YAW], Step[PITCH], Step[YAW], Step[ROLL]);
        }

        if (debugOrient)
        {
            print("Pitch_setpoint:%12.4f | Roll_setpoint:%12.4f\r\n", Rad2Deg(pitch_setpoint) / 1000, Rad2Deg(roll_setpoint) / 1000);
        }

        if (debugCnt)
        {
            print("Counter min %3d, %3d, %3d,  max %4d, %4d, %4d, count %3d, %3d, %3d, usbOverrun %4d\r\n",
                  MinCnt[ROLL], MinCnt[PITCH], MinCnt[YAW],
                  MaxCnt[ROLL], MaxCnt[PITCH], MaxCnt[YAW],
                  IrqCnt[ROLL], IrqCnt[PITCH], IrqCnt[YAW],
                  usbOverrun());
        }

        printcounter = 0;
    }

    LEDoff();
}

