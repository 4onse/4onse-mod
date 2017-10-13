#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include <RTClib.h>

#include <com/comunication.h>


String getFormattedDate(DateTime dt);
bool syncRTC(ICom& com, RTC_DS3231 rtc);
bool calcInterval(uint8_t current, uint8_t last, unsigned int interval);
bool calcSendTime(const DateTime& now, const DateTime& lastSend, const uint32_t sendingMinutes);

#endif
