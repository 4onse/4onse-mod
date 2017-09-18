
#include "functions.h"

String getFormattedDate(DateTime dt)
{
    char sz[30];
    sprintf(sz, "%04d-%02d-%02dT%02d:%02d:%02d", dt.year(), dt.month(), dt.day(), dt.hour(), dt.minute(), dt.second());
    delay(100);
    return String(sz) + String("+0000");
}


/**
 * Syncronize RTC
 */
bool syncRTC(ICom& com, RTC_DS1307 rtc) {

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

    if (tmp >= 5)
        return false;

    Serial.print("Year: ");
    Serial.println(result[0]);


    rtc.adjust(DateTime((uint16_t) result[0], (uint8_t) result[1], (uint8_t) result[2], (uint8_t) result[3], (uint8_t) result[4], (uint8_t) result[5]));
    Serial.println(F("done"));
    return true;
}

bool calcInterval(uint8_t current, uint8_t last, unsigned int interval)
{

    int8_t tmp = 0;

    if (current >= last){
        tmp = current - last;

    }else{
        tmp = 60 - last;
        tmp += current;
    }

    if ( tmp >= interval)
        return true;
    return false;

    // int8_t tmp = current - last;
    //
    // if (tmp < 0)
    //   tmp += 60;
    //
    // Serial.print(F("TMP: "));
    // Serial.println(tmp);
    //
    // if ( tmp >= interval)
    //     return true;
    // return false;
}
