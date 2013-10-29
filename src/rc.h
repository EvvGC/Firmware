/*
 *  rc.c
 *
 *  Created on: Aug 1, 2013
 *      Author: ala42
 *
 *  extracted code from main.c
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
