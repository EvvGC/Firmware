/*
 *  stopwatch.c
 *
 *  Created on: Aug 9, 2013
 *      Author: ala42
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
