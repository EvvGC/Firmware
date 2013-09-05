/*
 *  stopwatch.h
 *
 *  Created on: Aug 9, 2013
 *      Author: ala42
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
