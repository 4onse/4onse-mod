#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include <RTClib.h>

#include <istsos.h>
#include <com/comunication.h>
#include <RunningMedian.h>
#include <math.h>

extern uint8_t countSend;
extern bool sendStatus;


String getFormattedDate(const DateTime dt);
bool syncRTC(const ICom& com, const RTC_DS3231 rtc);
bool sendData(const Istsos& sos);
bool calcInterval(uint8_t current, uint8_t last, uint32_t interval);
bool calcLogInterval(const DateTime& current, const DateTime& last, uint8_t interval);
bool calcSendTime(const DateTime& now, const DateTime& lastSend, const uint32_t sendingMinutes);
bool calcSamplingTime(const DateTime& now, const DateTime& last, const uint8_t interval);

float getLastValue(const RunningMedian& median);

void checkMinVar(RunningMedian& median, float value, float variance);
void checkVar(RunningMedian& big, RunningMedian& minute, float variance);

#endif
