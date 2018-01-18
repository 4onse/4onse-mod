/******************************************************************************
    Copyright (C) 2012-2017, Istituto Scienze della Terra

    This program is free software; you can redistribute it and/or modify it
    under the terms of the GNU General Public License as published by the
    Free Software Foundation; either version 2 of the License, or (at your
    option) any later version.

    This program is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    General Public License for more details.

    You should have received a copy of the GNU General Public License along
    with this program; if not, write to the Free Software Foundation, Inc.,
    51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.

******************************************************************************/

#ifndef MEASURE_H
#define MEASURE_H

// #define DEBUG_MEASURES

#define MEASURE_OK 100
#define MEASURE_NOT_VALID 200
#define MEASURE_OUTSIDE_LIMITS 300
#define MEASURE_NO_DATA 400
#define MEASURE_FAIL_VARIANCE 500

#include <RunningMedian.h>

class Measure
{
    private:

        RunningMedian* minuteValue;
        RunningMedian* minuteFlag;

        RunningMedian* samplingValue;
        RunningMedian* samplingFlag;

        float minLimit;
        float maxLimit;

        float variance;
        float varianceBig;

        /**
        * This function calculate the flag of the [value]
        *
        * @param median     Array with values
        * @param flag       Array with flags
        * @param value      Value to calculate flag
        * @param variance   Variance
        *
        * @return uint16_t   Flag value
        */
        uint16_t checkMinuteVar(RunningMedian *median, RunningMedian *flag, const float value, const float variance);

        /**
        * Return last valid value from median
        *
        * @param  median    Array with value
        * @param  flag      Array with flag
        *
        * @return float     Last valid measure inside median
        */
        float getLastValue(RunningMedian *median, RunningMedian *flag);

        /**
        * Calc the median value and flag
        *
        * @return float[]   Array of 2 element that represent the measure with the QI Example: [18.27, 100]
        */
        float* calcAverageQI(RunningMedian *meas, RunningMedian *flag);

    public:
        /**
        * Class to manage measures
        *
        * @param length Number of measures per minute
        * @param lengthBIg NUmer of minute for each sampligPeriog
        */
        Measure(const uint8_t length=5, const uint8_t lengthBig=10, const float minLimit=-100.0, const float maxLimit=100.0, const float variance = 5, const float varianceBig = 10);

        /**
        * Add new mesure
        *
        * @param measure new measure to be added
        */
        void addMeasure(const float measure);

        /**
        * Calc the last minute value with it's quality index
        */
        void calcLastMin();

        /**
        * Calc the median value and flag and convert it to String
        *
        * @return String    string that represent the measure with the QI Example: 18.27:100
        */
        String getAverageQI();

};




#endif
