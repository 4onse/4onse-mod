// =========================================================================
//
// Copyright (C) 2012-2017, Istituto Scienze della Terra
//
// This program is free software; you can redistribute it and/or modify it
// under the terms of the GNU General Public License as published by the
// Free Software Foundation; either version 2 of the License, or (at your
// option) any later version.
//
// This program is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Public License for more details.
//
// You should have received a copy of the GNU General Public License along
// with this program; if not, write to the Free Software Foundation, Inc.,
// 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
//
// =========================================================================

#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include <RTClib.h>

#include <com/comunication.h>


String getFormattedDate(DateTime dt);
bool syncRTC(ICom& com, RTC_DS3231 rtc);
bool calcInterval(uint8_t current, uint8_t last, unsigned int interval);
bool calcSendTime(const DateTime& now, const DateTime& lastSend, const uint32_t sendingMinutes);
void alert(uint8_t blink);

#endif
