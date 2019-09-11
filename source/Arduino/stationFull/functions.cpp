#include "functions.h"

String getFormattedDate(const DateTime dt)
{
    char sz[30];
    sprintf(sz, "%04d-%02d-%02dT%02d:%02d:%02d", dt.year(), dt.month(), dt.day(), dt.hour(), dt.minute(), dt.second());
    delay(50);
    return String(sz) + String("+0000");
}

bool syncRTC(ICom& com, const RTC_DS3231 rtc)
{
    // Serial.print(F("Sincronize RTC..."));

    // this code is only for testing
    // rtc.adjust(DateTime((uint16_t) 2017, (uint8_t) 12, (uint8_t) 12, (uint8_t) 8, (uint8_t) 49, (uint8_t) 00));
    // return true;

    uint8_t tmp = 0;
    uint32_t* result = com.ntpUpdate("metasntp11.admin.ch", 0);

    //Serial.println(result[0]);
    while(result[0] < 2017 && tmp < 5)
    {
        result = com.ntpUpdate("metasntp11.admin.ch", 0);
        tmp++;
        delay(5000);
    }

    com.disconnect();

    if (tmp >= 5)
    {
        return false;
    }

    rtc.adjust(DateTime((uint16_t) result[0], (uint8_t) result[1], (uint8_t) result[2], (uint8_t) result[3], (uint8_t) result[4], (uint8_t) result[5]));
    // Serial.println(F("done"));
    delete[] result;

    return true;
}

uint8_t sendData(Istsos& sos)
{
    uint8_t res = sos.sendData();
    return res;
}

bool calcInterval(uint8_t current, uint8_t last, uint32_t interval)
{
    uint8_t tmp = 0;

    if (current >= last)
    {
        tmp = current - last;

    }else
    {
        tmp = 60 - last;
        tmp += current;
    }

    if ( tmp >= interval)
    {
        return true;
    }
    return false;

}

bool calcLogInterval(const DateTime& current, const DateTime& last, uint8_t interval)
{

    if ((current.minute() % 5) != 0)
    {
        return false;
    }

    uint32_t nowSec = current.secondstime() - current.second();
    uint32_t lastSec = last.secondstime() - last.second();

    uint32_t diff = nowSec - lastSec;

    if( diff >= (interval * 60))
    {
        return true;
    }

    return false;
}

bool calcSendTime(const DateTime& now, const DateTime& lastSend, const uint32_t sendingMinutes)
{
    uint32_t nowSec = now.secondstime() - now.second();
    uint32_t lastSec = lastSend.secondstime() - lastSend.second();
    uint32_t difference = nowSec - lastSec;
    uint32_t sendSec = sendingMinutes * 60;

    if (difference >= sendSec || (sendSec - difference) <= 8)
    {
        return true;
    }

    return false;
}

bool calcSamplingTime(const DateTime& now, const DateTime& last, const uint8_t interval)
{

    uint32_t nowSec = now.secondstime();
    uint32_t lastSec = last.secondstime();

    uint32_t diff = nowSec - lastSec;

    if (diff >= (uint32_t)(interval - 1))
    {
        return true;
    }
    return false;
}

float getLastValue(RunningMedian& median)
{
    uint8_t size = median.getCount();
    return median.getElement(size - 1);
}

void alert(uint8_t blink)
{
    for(uint8_t u = 0; u < 3; u++)
    {
        for(uint8_t i = 0; i < blink; i++)
        {
            digitalWrite(LED_BUILTIN, HIGH);
            delay(250);
            digitalWrite(LED_BUILTIN, LOW);
            delay(250);
        }

        delay(2000);
    }
}

void alert_long(uint8_t blink)
{
    for(uint8_t u = 0; u < 3; u++)
    {
        for(uint8_t i = 0; i < blink; i++)
        {
            digitalWrite(LED_BUILTIN, HIGH);
            delay(850);
            digitalWrite(LED_BUILTIN, LOW);
            delay(150);
        }

        delay(2000);
    }
}
