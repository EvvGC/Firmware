/*
 *  stopwatch.c
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

#include "systick.h"
#include "stopwatch.h"

unsigned int StopWatchInit(tStopWatch *sw)
{
    sw->tFirst = micros();
    sw->tLast = sw->tFirst;
    return sw->tLast;
}

unsigned int StopWatchNow(tStopWatch *sw)
{
    return sw->tLast;
}


unsigned int StopWatchLap(tStopWatch *sw)
{
    unsigned int tNow = micros();
    unsigned int tDelta = tNow - sw->tLast;
    sw->tLast = tNow;
    return tDelta;
}


unsigned int StopWatchTotal(tStopWatch *sw)
{
    return sw->tLast - sw->tFirst;
}
