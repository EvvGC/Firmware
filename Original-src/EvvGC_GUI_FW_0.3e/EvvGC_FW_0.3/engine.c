/*
 * 	engine.c
 *
 *	Created on: Jun 26, 2013
 *		Author: Denis aka caat
 */

#include "engine.h"
#include "stm32f10x_tim.h"
#include <math.h>
#include "pins.h"
#include "timers.h"
#include "adc.h"
#include "gyro.h"

float pitch, Gyro_Pitch_angle, pitch_output, pitch_setpoint=0, pitch_Error_current, pitch_Error_last, pitch_P, pitch_D, pitch_angle=0, pitch_angle_true, pitch_angle_correction;
float roll,   Gyro_Roll_angle,  roll_output,  roll_setpoint=0,  roll_Error_current,  roll_Error_last,  roll_P,  roll_D,  roll_angle=0,  roll_angle_correction;
float yaw,     Gyro_Yaw_angle,   yaw_output,   yaw_setpoint=0,   yaw_Error_current,   yaw_Error_last,   yaw_P,   yaw_D,   yaw_angle=0,   yaw_angle_correction;
float sinusas[91]={
0.000, 0.017, 0.035, 0.052, 0.070, 0.087, 0.105, 0.122, 0.139, 0.156,
0.174, 0.191, 0.208, 0.225, 0.242, 0.259, 0.276, 0.292, 0.309, 0.326, 
0.342, 0.358, 0.375, 0.391, 0.407, 0.423, 0.438, 0.454, 0.469, 0.485, 
0.500, 0.515, 0.530, 0.545, 0.559, 0.574, 0.588, 0.602, 0.616, 0.629, 
0.643, 0.656, 0.669, 0.682, 0.695, 0.707, 0.719, 0.731, 0.743, 0.755,
0.766, 0.777, 0.788, 0.799, 0.809, 0.819, 0.829, 0.839, 0.848, 0.857, 
0.866, 0.875, 0.883, 0.891, 0.899, 0.906, 0.914, 0.920, 0.927, 0.934, 
0.940, 0.945, 0.951, 0.956, 0.961, 0.966, 0.970, 0.974, 0.978, 0.982, 
0.985, 0.988, 0.990, 0.993, 0.995, 0.996, 0.998, 0.999, 0.999, 1.000,
1.000};

int n, m, tim_conf=0, YawPh1, YawPh2, YawPh3;
float dt=0.002, ADC1Ch1_vid, ADC1Ch13_vid, sinus, cosinus, ROLL, rc4_avg, gyroADC_ROLL_offset, gyroADC_PITCH_offset, gyroADC_YAW_offset, sukimas, gyroADC_y_last;
float acc_pitch_angle, acc_roll_angle, accADC_x, accADC_y, accADC_z, gyroADC_x, gyroADC_x_last, gyroADC_y, gyroADC_z, acc_pitch_angle_vid, acc_roll_angle_vid;
short int printcounter;

void pitch_PID(void)
{
//-------------------------------------PID Pitch-------------------------------
	
	pitch_Error_current = pitch_setpoint + pitch_angle*1000;        
	
	pitch_P = pitch_Error_current * (float)configData[0]/100;
	
	pitch_D = (float)configData[3]/100 * (pitch_Error_current - pitch_Error_last);
	pitch_Error_last = pitch_Error_current;
	
	
	pitch_output = (pitch_P  + pitch_D);
	    
	//set TIM1 values;
	TIM1->CCR1=(sin(pitch_output     )*5*configData[6])+500;
	TIM1->CCR2=(sin(pitch_output+2.09)*5*configData[6])+500;
	TIM1->CCR3=(sin(pitch_output+4.19)*5*configData[6])+500;      
} 

void roll_PID(void)
{
//-------------------------------------Roll Pitch-------------------------------
	
	roll_Error_current = roll_setpoint + roll_angle*1000;        
	
	roll_P = roll_Error_current * (float)configData[1]/100;
	
	roll_D = (float)configData[4]/100 * (roll_Error_current - roll_Error_last);
	roll_Error_last = roll_Error_current;
		
	roll_output = (roll_P  + roll_D);
	
	//set TIM8 values;    
	TIM8->CCR1=(sin(roll_output     )*5*configData[7])+500;
	TIM8->CCR2=(sin(roll_output+2.09)*5*configData[7])+500;
	TIM8->CCR3=(sin(roll_output+4.19)*5*configData[7])+500;       
} 

void yaw_PID(void)
{
//-------------------------------------Yaw Pitch-------------------------------
	
	yaw_Error_current = yaw_setpoint + yaw_angle*1000;        
	
	yaw_P = yaw_Error_current * (float)configData[2]/100;
	
	yaw_D = (float)configData[5]/100 * (yaw_Error_current - yaw_Error_last);
	yaw_Error_last = yaw_Error_current;
	
	
	yaw_output = (yaw_P  + yaw_D);
	
	// yaw_output=yaw_output+0.002;
	
	YawPh1=(sin(yaw_output     )*5*configData[8])+500;
	YawPh2=(sin(yaw_output+2.09)*5*configData[8])+500;
	YawPh3=(sin(yaw_output+4.19)*5*configData[8])+500;

	if(YawPh1>=930) YawPh1=930;
	if(YawPh2>=930) YawPh2=930;
	if(YawPh3>=930) YawPh3=930;	
	
	if(YawPh1<=10) YawPh1=10;
	if(YawPh2<=10) YawPh2=10;
	if(YawPh3<=10) YawPh3=10;	
	
	//set TIM4 values;
	TIM4->CCR1=YawPh1;
	TIM4->CCR2=YawPh2;
	TIM4->CCR3=YawPh3;
	
	YawPh1=YawPh1+70;
	YawPh2=YawPh2+70;
	YawPh3=YawPh3+70;
	
	if(YawPh1>=1000) YawPh1=1000;
	if(YawPh2>=1000) YawPh2=1000;
	if(YawPh3>=1000) YawPh3=1000;
	
	//set TIM5 values;
	TIM5->CCR1=YawPh1;
	TIM5->CCR2=YawPh2;
	TIM5->CCR3=YawPh3;      
} 


void engineProcess(void)
{
	LEDon;
	
	DEBUG_LEDoff;
	while(ConfigMode==1){TimerOff();}//Configuration loop
	
	MPU6050_ACC_get();//Getting Accelerometer data
	
	acc_roll_angle = -(atan2(accADC_x, accADC_z))+(configData[11]-50.00)*0.0035;   //Calculating pitch ACC angle+callibration
	acc_pitch_angle  = +(atan2(accADC_y, accADC_z));   //Calculating roll ACC angle		
	
	MPU6050_Gyro_get();//Getting Gyroscope data		
	
	acc_roll_angle_vid=  ((acc_roll_angle_vid * 99.00) + acc_roll_angle) / 100.00;	//Averaging pitch ACC values
	acc_pitch_angle_vid= ((acc_pitch_angle_vid * 99.00) + acc_pitch_angle) / 100.00; //Averaging roll  ACC values		
	
	sinus   = sinusas[(int)(rc4)];      //Calculating sinus		
	cosinus = sinusas[90-(int)(rc4)];   //Calculating cosinus
	
	ROLL =- gyroADC_z * sinus + gyroADC_y * cosinus;
	roll_angle = (roll_angle + ROLL * dt)    + 0.0002 * (acc_roll_angle_vid-roll_angle); //Roll Horizon
	
	
	//ROLL=-gyroADC_z*sinus+gyroADC_y*cosinus;
	yaw_angle =(yaw_angle + gyroADC_z * dt); //Yaw
	
	
	
	pitch_angle_true = ((pitch_angle_true  + gyroADC_x * dt) + 0.0002 * (acc_pitch_angle_vid - pitch_angle_true)); //Pitch Horizon
	sukimas = sukimas  + gyroADC_x * dt;
	ADC1Ch1_vid = ((ADC1Ch1_vid * 99.00) + (readADC1(1) / 4000.00)) / 100.00;	//Averaging ADC values
	ADC1Ch1_vid = 0.00;
	
	rc4_avg = ((rc4_avg * 499.00) + (rc4)) / 500.00;	//Averaging RC4 values
	pitch_angle = pitch_angle_true - rc4_avg / 57.3;//Adding angle
	
	pitch_angle_correction = pitch_angle * 150.0;
	if(pitch_angle_correction > 2.0)
	{
		pitch_angle_correction = 2.0;
	}
	if(pitch_angle_correction < -2.0)
	{
		pitch_angle_correction = -2.0;
	}
	pitch_setpoint = pitch_setpoint + pitch_angle_correction;//Pitch return to zero after collision
	
	roll_angle_correction = roll_angle * 150.0;
	if(roll_angle_correction > 2.0)
	{
		roll_angle_correction = 2.0;
	}
	if(roll_angle_correction < -2.0)
	{
		roll_angle_correction = -2.0;
	}
	roll_setpoint = roll_setpoint + roll_angle_correction;//Roll return to zero after collision
	
	
	
	ADC1Ch13_vid=  ((ADC1Ch13_vid * 99.00) + ((float)(readADC1(13) - 2000) / 4000.00)) / 100.00;	//Averaging ADC values		
	if(configData[10] == '0')
	{
		yaw_angle =(yaw_angle + gyroADC_z * dt) + 0.01 * (ADC1Ch13_vid - yaw_angle);
	} //Yaw AutoPan
	
	if(configData[10] == '1')
	{
		yaw_angle =(yaw_angle + gyroADC_z*dt);
	} //Yaw RCPan
	
	yaw_angle_correction = yaw_angle * 50.0;
	if(yaw_angle_correction > 1.0)
	{
		yaw_angle_correction = 1.0;
	}
	if(yaw_angle_correction < -1.0)
	{	
		yaw_angle_correction = -1.0;
	}
	yaw_setpoint=yaw_setpoint + yaw_angle_correction;//Yaw return to zero after collision
	
	if(tim_conf == 0)
	{
		//rewrite that thing;
		Timer1_Config(); 
		Timer8_Config();	
		Timer5_Config(); 
		Timer4_Config(); 
		tim_conf = 1; 	
		TIM_Cmd(TIM5, ENABLE);
		TIM_CtrlPWMOutputs(TIM5, ENABLE);
		for (n=1 ; n<4 ; n++) ; //small delay before starting Timer4
		TIM_Cmd(TIM4, ENABLE);
		TIM_CtrlPWMOutputs(TIM4, ENABLE);
	}
	
	pitch_PID();//Pitch axis pid
	roll_PID(); //Roll axis pid
	yaw_PID(); //Yaw axis pid
	
	
	printcounter++; //Print data to UART
	if (printcounter>=50)
	{
		//sprintf (buff, " %d %d %c Labas\n\r", ACCread[0], ACCread[1], ACCread[2]);
		//sprintf (buff, " %x %x %x %x %x %x Labas\n\r", ACCread[0], ACCread[1], ACCread[2], ACCread[3], ACCread[4], ACCread[5]);
		//if((int)GYROread[0]>50 || (int)GYROread[0]<40){
		//	sprintf (buff, "%d\n\r", (InputLevel[0]+InputLevel[1]+InputLevel[2]+InputLevel[3]+InputLevel[4]+InputLevel[5]+InputLevel[6]+InputLevel[7]+InputLevel[8]));
		//sprintf (buff, "%d \n\r", gyroADC_PITCH);
		//sprintf (buff, "Labas %f %f %f \n\r", accADC_x, accADC_y, accADC_z);
		//sprintf (buff, "%3.1f %3.1f \n\r", acc_roll_angle_vid*57.3,  acc_pitch_angle_vid *57.3);
		//sprintf (buff, "%3.1f \n\r", pitch_angle*57.3);
		//sprintf (buff, "%3.1f \n\r", sukimas*57.3);
		//sprintf (buff, "%d\n\r", I2Cerrorcount);
		//USART_PutString(buff);
		printcounter=0;
	}
	
	stop=0;
	LEDoff;	
	watchcounter=0;	
}

