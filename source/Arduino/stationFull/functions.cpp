
#include "functions.h"

String getFormattedDate(DateTime dt)
{
    char sz[30];
    sprintf(sz, "%04d-%02d-%02dT%02d:%02d:%02d", dt.year(), dt.month(), dt.day(), dt.hour(), dt.minute(), dt.second());
    delay(50);
    return String(sz) + String("+0000");
}


/**
 * Syncronize RTC
 */
bool syncRTC(ICom& com, RTC_DS3231 rtc)
{
    Serial.print(F("Sincronize RTC..."));

    // this code is only for testing
    // rtc.adjust(DateTime((uint16_t) 2017, (uint8_t) 9, (uint8_t) 13, (uint8_t) 12, (uint8_t) 00, (uint8_t) 00));
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

    uint16_t year = (uint16_t) result[0];
    uint8_t month = (uint8_t) result[1];
    uint8_t day = (uint8_t) result[2];

    rtc.adjust(DateTime(year, month, day, (uint8_t) result[3], (uint8_t) result[4], (uint8_t) result[5]));
    delay(2000);

    // check that the rtc has the correct date
    DateTime now = rtc.now();

    if(now.year() == year && now.month() == month && now.day() == day)
    {
        Serial.println(F("done"));
        return true;
    }
    return false;
}

bool calcInterval(uint8_t current, uint8_t last, unsigned int interval)
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

bool calcSendTime(const DateTime& now, const DateTime& lastSend, const uint32_t sendingMinutes)
{
    uint32_t nowSec = now.secondstime();
    uint32_t lastSec = lastSend.secondstime();
    uint32_t difference = nowSec - lastSec;
    uint32_t sendSec = sendingMinutes * 60;

    if (difference >= sendSec || (sendSec - difference) <= 8)
    {
        return true;
    }

    return false;

}

void alert(uint8_t blink)
{
    for(uint8_t i = 0; i < blink; i++)
    {
        digitalWrite(LED_BUILTIN, LOW);
        delay(250);
        digitalWrite(LED_BUILTIN, HIGH);
        delay(250);
    }

    delay(2000);
}
