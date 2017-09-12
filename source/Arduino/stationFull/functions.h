#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include <RTClib.h>

#include <com/comunication.h>


String getFormattedDate(DateTime dt);
bool syncRTC(ICom& com, RTC_DS1307 rtc);
bool calcInterval(uint8_t current, uint8_t last, unsigned int interval);


#endif
