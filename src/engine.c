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
#include "i2c.h"
#include "definitions.h"
#include "usb.h"
#include "main.h"
#include "usb_lib.h"

int debugPrint   = 0;
int debugPerf    = 0;
int debugSense   = 0;
int debugCnt     = 0;
int debugRC      = 0;
int debugOrient  = 0;
int debugAutoPan = 0;

struct sTraceBuffer g_TraceBuffer;
int g_bTraceBufferReady = 0;

float /*pitch, Gyro_Pitch_angle,*/ pitch_setpoint = 0.0f, pitch_Error_last = 0.0f,  pitch_angle_correction;
float /*roll,  Gyro_Roll_angle,*/  roll_setpoint  = 0.0f,  roll_Error_last = 0.0f,   roll_angle_correction;
float /*yaw,   Gyro_Yaw_angle,*/   yaw_setpoint   = 0.0f,   yaw_Error_last = 0.0f,    yaw_angle_correction;

float ADC1Ch13_yaw;

static float rollRCOffset = 0.0f, pitchRCOffset = 0.0f, yawRCOffset = 0.0f;

static int printcounter = 0;

float Output[EULAR];

float CameraOrient[EULAR];
float AccAngleSmooth[EULAR];

float AccData[NUMAXIS]  = {0.0f, 0.0f, 0.0f};
float GyroData[NUMAXIS] = {0.0f, 0.0f, 0.0f};

float Step[NUMAXIS]     = {0.0f, 0.0f, 0.0f};
float RCSmooth[NUMAXIS] = {0.0f, 0.0f, 0.0f};

void roll_PID(void)
{
    float Error_current = roll_setpoint + CameraOrient[ROLL] * 1000.0;
    float KP = Error_current * ((float)configData[1] / 1000.0);
    float KD = ((float)configData[4] / 100.0) * (Error_current - roll_Error_last);

    roll_Error_last = Error_current;

    Output[ROLL] = KD + KP;
    SetRollMotor(KP + KD, configData[7]);
}

void pitch_PID(void)
{
    float Error_current = pitch_setpoint + CameraOrient[PITCH] * 1000.0;
    float KP = Error_current * ((float)configData[0] / 1000.0);
    float KD = ((float)configData[3] / 100.0) * (Error_current - pitch_Error_last);

    pitch_Error_last = Error_current;

    Output[PITCH] = KD + KP;
    SetPitchMotor(KP + KD, configData[6]);
}

void yaw_PID(void)
{
    float Error_current = yaw_setpoint + CameraOrient[YAW] * 1000.0;
    float KP = Error_current * ((float)configData[2] / 1000.0);
    float KD = ((float)configData[5] / 100.0) * (Error_current - yaw_Error_last);

    yaw_Error_last = Error_current;

    Output[YAW] = KD + KP;
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

    for (i = 0; i < init_loops; i++)
    {
        MPU6050_ACC_get(AccData); //Getting Accelerometer data

        AccAngle[ROLL]  = -(atan2f(AccData[X_AXIS], AccData[Z_AXIS]));   //Calculating pitch ACC angle
        AccAngle[PITCH] = +(atan2f(AccData[Y_AXIS], AccData[Z_AXIS]));   //Calculating roll ACC angle

        AccAngleSmooth[ROLL]  = ((AccAngleSmooth[ROLL] * (float)(init_loops - 1))  + AccAngle[ROLL])  / (float)init_loops; //Averaging pitch ACC values
        AccAngleSmooth[PITCH] = ((AccAngleSmooth[PITCH] * (float)(init_loops - 1)) + AccAngle[PITCH]) / (float)init_loops; //Averaging roll  ACC values
        Delay_ms(1);
    }

    CameraOrient[PITCH] = AccAngleSmooth[PITCH];
    CameraOrient[ROLL] = AccAngleSmooth[ROLL];
    CameraOrient[YAW] = 0.0f;

    g_bTraceBufferReady = 0;
    g_TraceBuffer.ui32Counter = 0;
}

void Get_Orientation(float *SmoothAcc, float *Orient, float *AccData, float *GyroData, float dt)
{
    float AccAngle[EULAR];
    float GyroRate[EULAR];

    AccAngle[ROLL]  = -(atan2f(AccData[X_AXIS], AccData[Z_AXIS]));   //Calculating pitch ACC angle
    AccAngle[PITCH] = +(atan2f(AccData[Y_AXIS], AccData[Z_AXIS]));   //Calculating roll ACC angle

    SmoothAcc[ROLL]  = ((SmoothAcc[ROLL] * 99.0f)  + AccAngle[ROLL])  / 100.0f; //Averaging pitch ACC values
    SmoothAcc[PITCH] = ((SmoothAcc[PITCH] * 99.0f) + AccAngle[PITCH]) / 100.0f; //Averaging roll  ACC values

    GyroRate[PITCH] =  GyroData[X_AXIS];
    Orient[PITCH]   = (Orient[PITCH] + GyroRate[PITCH] * dt) + 0.0002f * (SmoothAcc[PITCH] - Orient[PITCH]);  //Pitch Horizon

    GyroRate[ROLL] = -GyroData[Z_AXIS] * sinf(Orient[PITCH]) + GyroData[Y_AXIS] * cosf(fabsf(Orient[PITCH]));
    Orient[ROLL]   = (Orient[ROLL] + GyroRate[ROLL] * dt)    + 0.0002f * (SmoothAcc[ROLL] - Orient[ROLL]); //Roll Horizon

    GyroRate[YAW] = -GyroData[Z_AXIS] * cosf(fabsf(Orient[PITCH])) - GyroData[Y_AXIS] * sinf(Orient[PITCH]); //presuming Roll is horizontal
    Orient[YAW]   = (Orient[YAW] + GyroRate[YAW] * dt); //Yaw
}

//---------------------YAW autopan----------------------//
//#define ANGLE2SETPOINT -1000
#define DEADBAND 2.0f //in radians with respect to one motor pole (actual angle is (DEADBAND / numberPoles) * R2D)
#define MOTORPOS2SETPNT 0.35f //scaling factor for how fast it should move
#define AUTOPANSMOOTH 40.0f
//#define LPFTIMECONSTANT 20 //change this to adjust sensitivity

//float yawAngleLPF=0;
float centerPoint = 0.0f;
float stepSmooth  = 0.0f;
float step        = 0.0f;

float autoPan(float motorPos, float setpoint)
{

    if (motorPos < centerPoint - DEADBAND)
    {
        centerPoint = (+DEADBAND);
        step = MOTORPOS2SETPNT * motorPos; //dampening
    }
    else if (motorPos > centerPoint + DEADBAND)
    {
        centerPoint = (-DEADBAND);
        step = MOTORPOS2SETPNT * motorPos; //dampening
    }
    else
    {
        step = 0.0f;
        centerPoint = 0.0f;
    }
    stepSmooth = (stepSmooth * (AUTOPANSMOOTH - 1.0f) + step) / AUTOPANSMOOTH;
    return (setpoint -= stepSmooth);
}
//--------------------Engine Process-----------------------------//
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
    //pitch_setpoint += Step[PITCH];
    pitchRCOffset += Step[PITCH] / 1000.0;

    pitch_angle_correction = constrain((CameraOrient[PITCH] + pitchRCOffset) * R2D, -CORRECTION_STEP, CORRECTION_STEP);
    pitch_setpoint += pitch_angle_correction; // Pitch return to zero after collision

    // Roll Adjustments
    //roll_setpoint += Step[ROLL];
    rollRCOffset += Step[ROLL] / 1000.0;

    // include the config roll offset which is scaled to 0 = -10.0 degrees, 100 = 0.0 degrees, and 200 = 10.0 degrees
    roll_angle_correction = constrain((CameraOrient[ROLL] + rollRCOffset + Deg2Rad((configData[11] - 100) / 10.0)) * R2D, -CORRECTION_STEP, CORRECTION_STEP);
    roll_setpoint += roll_angle_correction; //Roll return to zero after collision

    // if we enabled AutoPan on Yaw
    if (configData[10] == '0')
    {
        //ADC1Ch13_yaw = ((ADC1Ch13_yaw * 99.0) + ((float)(readADC1(13) - 2000) / 4000.0)) / 100.0;  // Average ADC value
        //CameraOrient[YAW] = CameraOrient[YAW] + 0.01 * (ADC1Ch13_yaw - CameraOrient[YAW]);
        yaw_setpoint = autoPan(Output[YAW], yaw_setpoint);
    }
    else
    {
        // Yaw Adjustments
        yaw_setpoint += Step[YAW];
        yawRCOffset += Step[YAW] / 1000.0;
    }

#if 0
    yaw_angle_correction = constrain((CameraOrient[YAW] + yawRCOffset) * R2D, -CORRECTION_STEP, CORRECTION_STEP);
    yaw_setpoint += yaw_angle_correction; // Yaw return to zero after collision
#endif

    unsigned long tCalc = StopWatchLap(&sw);

    pitch_PID();
    roll_PID();
    yaw_PID();

    unsigned long tPID = StopWatchLap(&sw);
    unsigned long tAll = StopWatchTotal(&sw);

    printcounter++;

    g_TraceBuffer.ui32Counter++;
    g_TraceBuffer.fAccX = AccData[X_AXIS];
    g_TraceBuffer.fAccY = 42 + AccData[Y_AXIS];
    g_TraceBuffer.fAccZ = AccData[Z_AXIS];
    g_bTraceBufferReady = 1;
//	if(g_bTraceBufferReady){			// TODO: check if data has been sent
	if(GetTxStallStatus(0x82)){			// TODO: check if data has been sent
		int sendLength = sizeof(g_TraceBuffer);
	    UserToPMABufferCopy((uint8_t *)&g_TraceBuffer, ENDP2_TXADDR, sendLength);
	    SetEPTxCount(ENDP2, sendLength);
	    SetEPTxValid(ENDP2);
		g_bTraceBufferReady = 0;
	}

    //if (printcounter >= 500 || dt > 0.0021)
    if (printcounter >= 500)
    {
        if (debugPrint){
        	print(">>%d, %d\n", g_TraceBuffer.ui32Counter, g_bTraceBufferReady);
        }
        if (debugPrint)
        {
            print("Loop: %7d, I2CErrors: %d, angles: roll %7.2f, pitch %7.2f, yaw %7.2f\r\n",
                  loopCounter, I2Cerrorcount, Rad2Deg(CameraOrient[ROLL]),
                  Rad2Deg(CameraOrient[PITCH]), Rad2Deg(CameraOrient[YAW]));
        }

        if (debugSense)
        {
            print(" dt %f, AccData: %8.3f | %8.3f | %8.3f, GyroData %7.3f | %7.3f | %7.3f \r\n",
                  dt, AccData[X_AXIS], AccData[Y_AXIS], AccData[Z_AXIS], GyroData[X_AXIS], GyroData[Y_AXIS], GyroData[Z_AXIS]);
        }

        if (debugPerf)
        {
            print("idle: %5.2f%%, time[�s]: attitude est. %4d, IMU acc %4d, gyro %4d, angle %4d, calc %4d, PID %4d\r\n",
                  GetIdlePerf(), tAll, tAccGet, tGyroGet, tAccAngle, tCalc, tPID);
        }

        if (debugRC)
        {
            print(" RC2avg: %7.2f |  RC4avg: %7.2f |  RC3avg: %7.2f | RStep:%7.3f  PStep: %7.3f  YStep: %7.3f\r\n",
                  RCSmooth[ROLL], RCSmooth[PITCH], RCSmooth[YAW], Step[ROLL], Step[PITCH], Step[YAW]);
        }

        if (debugOrient)
        {
            print("Roll_setpoint:%12.4f | Pitch_setpoint:%12.4f | Yaw_setpoint:%12.4f\r\n",
                  roll_setpoint, pitch_setpoint, yaw_setpoint);
        }

        if (debugCnt)
        {
            print("Counter min %3d, %3d, %3d,  max %4d, %4d, %4d, count %3d, %3d, %3d, usbOverrun %4d\r\n",
                  MinCnt[ROLL], MinCnt[PITCH], MinCnt[YAW],
                  MaxCnt[ROLL], MaxCnt[PITCH], MaxCnt[YAW],
                  IrqCnt[ROLL], IrqCnt[PITCH], IrqCnt[YAW],
                  usbOverrun());
        }

        if (debugAutoPan)
        {
            print("Pitch_output:%3.2f | Roll_output:%3.2f | Yaw_output:%3.2f | centerpoint:%4.4f\n\r",
                  Output[PITCH],
                  Output[ROLL],
                  Output[YAW],
                  centerPoint);
        }

        printcounter = 0;
    }

    LEDoff();
}

