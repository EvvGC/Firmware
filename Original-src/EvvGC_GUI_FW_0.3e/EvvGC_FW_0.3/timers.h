/*
 * 	timers.h
 *
 *	Created on: Jun 25, 2013
 *		Author: Denis aka caat
 */

#ifndef TIMERS_H_
#define TIMERS_H_

#define TIM2_PRESCALER	71
#define TIM2_AUTORELOADVALUE	2000

void Timer1_Config(void);//roll timer config
void Timer2_Config(void);//main loop timer config
void Timer3_Config(void);//RC control timer config
void Timer4_Config(void);//yaw timer config
void Timer5_Config(void);//yaw timer config
void Timer8_Config(void);//pitch timer config

void TimerOff(void);

#endif /* TIMERS_H_ */
