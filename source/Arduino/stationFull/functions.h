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

#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include <RTClib.h>

#include <istsos.h>
#include <com/comunication.h>
#include <RunningMedian.h>
#include <math.h>

extern uint8_t countSend;
extern bool sendStatus;

/**
 * Return string from date object
 *
 * @param dt date to convert to string
 *
 * @return String String date
 */
String getFormattedDate(const DateTime dt);

/**
 * Syncronize RTC
 *
 * @param com communication object
 * @param rtc rtc object
 *
 * @return rtc sync status
 */
bool syncRTC(ICom& com, const RTC_DS3231 rtc);

/**
 * Send data procedure
 *
 * @param sos   sos object
 *
 * @return bool send status
*/
uint8_t sendData(Istsos& sos);

/**
 * Check if it's time to do the computation on the last minutes measures
 *
 * @param current
 * @param last
 * @param interval
 *
 * @return bool
*/
bool calcInterval(uint8_t current, uint8_t last, uint32_t interval);

/**
 * Check if it's time log data on the sd card
 *
 * @param current   current datetime
 * @param last      last log datetime
 * @param interval  intervals in minutes
 *
 * @return bool
*/
bool calcLogInterval(const DateTime& current, const DateTime& last, uint8_t interval);

/**
 * Check if it's time to send data
 *
 * @param now               current datetime
 * @param lastSend          last send datetime
 * @param sendingMinutes    intervals in mimutes
 *
 * @return bool
*/
bool calcSendTime(const DateTime& now, const DateTime& lastSend, const uint32_t sendingMinutes);

/**
 * Check if it's time to do the computation on the last minutes measures
 *
 * @param now       current datetime
 * @param last      last sampling datetime
 * @param interval  intervals in seconds
 *
 * @return bool
*/
bool calcSamplingTime(const DateTime& now, const DateTime& last, const uint8_t interval);

/**
 * Get last value from a RunningMedian object
 *
 * @param median    RunningMedian object
 *
 * @return float    Last value
*/
float getLastValue(RunningMedian& median);

/**
*    Function to blink the buildin led a defined amount of time
*
*   @param blink    Number of time to blink
*/
void alert(uint8_t blink);

#endif
