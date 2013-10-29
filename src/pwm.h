/*
 *  pwm.h
 *
 *  Created on: Aug 1, 2013
 *      Author: ala42
 */

/*
    Original work Copyright (c) 2012 [Evaldis - RCG user name]
    Modified work Copyright 2012 Alan K. Adamson

    This file is part of EvvGC.

    EvvGC is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    EvvGC is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with EvvGC.  If not, see <http://www.gnu.org/licenses/>.

*/

#ifndef PWM_H_
#define PWM_H_

#define PWM_PERIODE 1000

typedef enum
{
    ROLL,
    PITCH,
    YAW,
    NUMAXIS
} tAxis;

extern int MaxCnt[NUMAXIS];
extern int MinCnt[NUMAXIS];
extern int IrqCnt[NUMAXIS];

extern int timer_4_5_deadtime_delay;
extern float testPhase;

void MaxCntClear(void);
void SetRollMotor(float phi, int power);
void SetPitchMotor(float phi, int power);
void SetYawMotor(float phi, int power);

void PWMOff(void);
void PWMConfig(void);

#endif /* PWM_H_ */
