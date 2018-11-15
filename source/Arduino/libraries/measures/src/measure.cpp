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
#include "measure.h"


Measure::Measure(const uint8_t length, const uint8_t lengthBig, const float minLimit, const float maxLimit, const float variance, const float varianceBig)
{

    this->variance = variance;
    this->varianceBig = varianceBig;

    this->minLimit = minLimit;
    this->maxLimit = maxLimit;

    this->minuteValue = new RunningMedian(length);
    this->minuteFlag = new RunningMedian(length);

    this->samplingValue = new RunningMedian(lengthBig);
    this->samplingFlag = new RunningMedian(lengthBig);
}

void Measure::addMeasure(const float measure)
{
    uint16_t flag = 0;
    if(measure >= this->minLimit && measure <= this->maxLimit)
    {
        flag = this->checkMinuteVar(this->minuteValue, this->minuteFlag, measure, this->variance);
    }
    else
    {
        flag = MEASURE_OUTSIDE_LIMITS;
    }

    this->minuteValue->add(measure);
    this->minuteFlag->add(flag);


    #ifdef DEBUG_MEASURES
        uint8_t size = this->minuteValue->getCount();
        Serial.print(F("minute status: "));
        for( int i = 0; i < size; i++)
        {
            Serial.print(this->minuteValue->getElement(i));
            Serial.print(":");
            Serial.print((uint16_t)this->minuteFlag->getElement(i));
            Serial.print(";");
        }
        Serial.println(F(""));
    #endif


}

uint16_t Measure::checkMinuteVar(RunningMedian *median, RunningMedian *flag, const float value, const float variance)
{
    if (isnan(value))
    {
        return MEASURE_NO_DATA;
    }

    if(median->getCount() != 0)
    {
        float lastVal = this->getLastValue(median, flag);

        if(isnan(lastVal))
        {
            return MEASURE_OK;
        }
        if(fabs(lastVal - value) <= variance)
        {
            return MEASURE_OK;
        }
        else
        {
            #ifdef DEBUG_MEASURES
                Serial.print(F("Failed variance test, lastValue:"));
                Serial.print(lastVal);
                Serial.print(F(" current value: "));
                Serial.print(value);
                Serial.print(F(" variance: "));
                Serial.println(variance);
            #endif
            return MEASURE_FAIL_VARIANCE;
        }
    }
    else
    {
        return MEASURE_OK;
    }
}

float Measure::getLastValue(RunningMedian *median, RunningMedian *flag)
{
    uint8_t size = median->getCount();
    for( uint8_t i = size; i > 0; i--)
    {
        if((uint16_t) flag->getElement(i) == MEASURE_OK)
        {
            return median->getElement(i);
        }
    }

    return NAN;
}

float* Measure::calcAverageQI(RunningMedian *meas, RunningMedian *flag)
{
    RunningMedian* tmp = new RunningMedian(meas->getSize());
    float* result = new float[2];

    uint8_t length = meas->getCount();

    // take only valid values
    for(int i = 0; i < length; i++)
    {
        if(flag->getElement(i) == MEASURE_OK)
        {
            tmp->add(meas->getElement(i));
        }
    }

    // check data
    uint8_t tmpLength = tmp->getCount();

    float check = (float) ((float)tmpLength / (float)length);

    if( check >= 0.666)
    {
        result[1] = MEASURE_OK;
    }
    else
    {
        #ifdef DEBUG_MEASURES
            Serial.print(F("Length: "));
            Serial.println(check);
        #endif
        result[1] = MEASURE_NOT_VALID;
    }

    result[0] = tmp->getAverage();

    delete tmp;

    return result;
}

void Measure::calcLastMin()
{
    float* result = this->calcAverageQI(this->minuteValue, this->minuteFlag);
    uint16_t flag = 0;

    if(result[1] == MEASURE_OK)
    {
        flag = this->checkMinuteVar(this->samplingValue, this->samplingFlag, result[0], this->varianceBig);
    }
    else
    {
        flag = (uint16_t) result[1];
    }

    this->samplingValue->add(result[0]);
    this->samplingFlag->add(flag);

    #ifdef DEBUG_MEASURES
        uint8_t size = this->samplingValue->getCount();
        Serial.print(F("    period status: "));
        for( int i = 0; i < size; i++)
        {
            Serial.print(this->samplingValue->getElement(i));
            Serial.print(",");
            Serial.print((uint16_t)this->samplingFlag->getElement(i));
            Serial.print(";");
        }
        Serial.println(F(""));
    #endif

    this->minuteValue->clear();
    this->minuteFlag->clear();
    delete[] result;

}

String Measure::getAverageQI()
{
    float* result = this->calcAverageQI(this->samplingValue, this->samplingFlag);

    this->samplingValue->clear();
    this->samplingFlag->clear();

    if(isnan(result[0]))
    {
        result[0] = 0;
    }

    String tmp = String(result[0]) + ":" + String((uint16_t)result[1]);

    delete[] result;

    return tmp;
}
