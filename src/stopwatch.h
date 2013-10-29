/*
 *  stopwatch.h
 *
 *  Created on: Aug 9, 2013
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

#ifndef STOPWATCH_H_
#define STOPWATCH_H_

typedef struct
{
    unsigned int tFirst;
    unsigned int tLast;
} tStopWatch;

unsigned int StopWatchInit(tStopWatch *sw);
unsigned int StopWatchNow(tStopWatch *sw);
unsigned int StopWatchLap(tStopWatch *sw);
unsigned int StopWatchTotal(tStopWatch *sw);

#endif /* STOPWATCH_H_ */
