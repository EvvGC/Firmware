/*
 *  rc.c
 *
 *  Created on: Aug 1, 2013
 *      Author: ala42
 *
 *  extracted code from main.c
 */

#ifndef RC_H_
#define RC_H_

#define DEAD_ZONE 20.0F
#define RC_CENTER_VAL 500.0F
#define RC2STEP 0.002

void RC_Config(void);
int GetAUX3(void);
int GetAUX4(void);
int GetAUX2(void);
void Get_RC_Step(float *Step, float *RCSmooth);

#endif /* RC_H_ */
