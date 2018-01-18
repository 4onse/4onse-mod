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

#define DEBUG
#define WIND
// istsos comunication library (GPRS)
#include <istsos.h>
#include <com/drok.h>
#include <log/sdOpenlog.h>

// temperature
#include <OneWire.h>
#include <DallasTemperature.h>

// pressure/humidity
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

// light
#include <BH1750.h>
// RTC
#include <RTClib.h>
// internal temp
#include <DHT.h>
// Median
#include <RunningMedian.h>
#include <measure.h>

// Include function library
#include "functions.h"

/*****************************************
 * Configure params
 ****************************************/
// #define SERVER "geoservice.ist.supsi.ch"
// #define URI "/4onse/wa/istsos/services/sos/operations/fastinsert"
// #define PROCEDURE_ID "175378da9a0511e79e2008002745029a"
// #define BASIC_AUTH "YWRtaW46QlYzWGp2clA="
//
// #define APN "gprs.swisscom.ch"
// #define APNUSER "gprs"
// #define PASS "gprs"
// #define SIM_PIN ""

/*****************************************
 * Define sensor pin
 ****************************************/
#define ONE_WIRE_BUS 9
#define SOIL_A_PIN A13


#define DHTPIN 10
#define DHTTYPE DHT11
#define FAN_PIN 12

#define WSPEED 3
#define RAIN 2
#define WDIR A0

// BME could be 0x76 or 0x77
uint8_t BME_I2C_ADDR = 0x76;

// magic
#define SF(x) String(F(x))

/*****************************************
 * Define time intervals
 ****************************************/

#define SAMPLING_TIME_MIN 5     ///< Sampling logging time (minutes)
#define SENDING_TIME_MIN 15     ///< Sending time (minutes)

// how many measures every minutes
#define SAMPLING_TIME_MED 6
//
#define SAMPLING_TIME_MEDIAN SAMPLING_TIME_MIN


/*****************************************
 * Sensors definition
 ****************************************/
DHT dht(DHTPIN, DHTTYPE);
BH1750 lightMeter;
Adafruit_BME280 bme;
RTC_DS3231 rtc;
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature dstemp(&oneWire);



/*****************************************
 * Comunication and logging system
 ****************************************/
Drok com = Drok(Serial1, APN, APNUSER, PASS, BASIC_AUTH, SIM_PIN);
OpenLog sdLog = OpenLog(Serial2);
Istsos sos(sdLog, com, SERVER, URI, PROCEDURE_ID);

/******************************************
 * Global variable FOR INTERRUPTS
 *****************************************/

volatile float lastrain = 0;
volatile unsigned long rainlast;
float rain = 0.0;

#ifdef WIND
    uint8_t windClicks = 0;
    volatile unsigned long lastWindCheck = 0;
    volatile unsigned long lastWindIRQ = 0;

    /******************************************
     * Arrays to get the median values last minute
     *****************************************/
    RunningMedian medianWinDir  = RunningMedian(SAMPLING_TIME_MED);
    RunningMedian medianWinSp   = RunningMedian(SAMPLING_TIME_MED);
    /******************************************
     * Arrays to get the median values last minute
     *****************************************/
    RunningMedian SampMedianWinDir  = RunningMedian(SAMPLING_TIME_MEDIAN);
    RunningMedian SampMedianWinSp   = RunningMedian(SAMPLING_TIME_MEDIAN);
#endif

Measure measTemp     = Measure(SAMPLING_TIME_MED, SAMPLING_TIME_MEDIAN, -80.0, 60.0, 2.0, 3.0);
Measure measHum      = Measure(SAMPLING_TIME_MED, SAMPLING_TIME_MEDIAN, 0.0, 100.0, 5.0, 10.0);
Measure measPres     = Measure(SAMPLING_TIME_MED, SAMPLING_TIME_MEDIAN, 500.0, 1100.0, 0.3, 0.5);
Measure measSoil     = Measure(SAMPLING_TIME_MED, SAMPLING_TIME_MEDIAN, 0.0, 100.0, 5.0, 10.0);
Measure measLux      = Measure(SAMPLING_TIME_MED, SAMPLING_TIME_MEDIAN, 0.0, 100000.0, 1000, 2000);
Measure measIntTemp  = Measure(SAMPLING_TIME_MED, SAMPLING_TIME_MEDIAN, -80.0, 60.0, 2.0, 3.0);
Measure measIntHum   = Measure(SAMPLING_TIME_MED, SAMPLING_TIME_MEDIAN, 0.0, 100.0, 5.0, 10.0);


// Variable to manage data log and data send
uint8_t lastMin = 0;
uint8_t lastDay = 0;
DateTime lastLogDate;
uint8_t lastMisMin = 0;
DateTime lastSendDate;
DateTime lastSamplingDate;

uint8_t countSend = 0;
bool sendStatus = true;

/******************************************
 * Function to monitor RAM usage
 *****************************************/
int freeRam() {
    extern int __heap_start, *__brkval;
    int v;
    return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Hardware interrupt
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

/**
*  Count rain gauge bucket tips as they occur
*  activated by the magnet and reed switch in the rain gauge, attached to input D2
*/
void rainIRQ()
{
    if ((unsigned long)(millis() - rainlast) > 10) // ignore switch-bounce glitches less than 10mS after initial edge
    {
        lastrain += 0.2; // 0.011;
        rainlast = millis(); // set up for next event
    }
}
#ifdef WIND
    /**
    *   Activated by the magnet in the anemometer (2 ticks per rotation), attached to input D3
    */
    void wspeedIRQ()
    {
        if ((unsigned long) (millis() - lastWindIRQ) > 10)
        {
            lastWindIRQ = millis();
            windClicks++; //There is 1.492MPH for each click per second.
        }
    }

    /**
     * get wind speed
     *
     * @return float wind speed (m/s)
     */
    float get_wind_speed()
    {
        unsigned long now = millis();
        float deltaTime = (float) now - lastWindCheck; //750ms

        deltaTime /= 1000.0; //Covert to seconds

        float windSpeed = (float)(windClicks / deltaTime); //3 / 0.750s = 4

        windClicks = 0; //Reset and start watching for new wind
        lastWindCheck = now;

        windSpeed = (windSpeed * 1.492) * (1.609344 / 3.6); //4 * 1.492 = 5.968MPH

        return (windSpeed);
    }
    /**
       Read the wind direction sensor, return heading in degrees
     */
    short get_wind_direction()
    {
        unsigned short adc;
        adc = analogRead(WDIR); // get the current reading from the sensor

        // https://www.sparkfun.com/datasheets/Sensors/Weather/Weather%20Sensor%20Assembly.pdf
        // using 10K pull-up resistor

        if (adc < 100) return (90);
        if (adc < 200) return (135);
        if (adc < 300) return (180);
        if (adc < 500) return (45);
        if (adc < 660) return (225);
        if (adc < 800) return (0);
        if (adc < 946) return (315);
        if (adc < 979) return (270);
        return -1;
    }
#endif

/**
*   Log message inside SD
*   @param message
*/
void logMessage(const String message)
{
    String date = getFormattedDate(rtc.now());
    sos.logging(date, message);

    #ifdef DEBUG
        Serial.print(date + " : ");
        Serial.println(message);
    #endif
}

/**
    Initialization
*/
void setup() {

    // init serial port
    Serial.begin(9600);     // Serial to PC
    while(!Serial){}
    Serial1.begin(57200);   // Serial to GPRS
    while(!Serial1){}
    Serial2.begin(9600);    // Serial to SD
    while(!Serial2){}
    delay(3000);

    // this variables are used in function.cpp
    countSend = 0;
    sendStatus = true;

    // check rtc
    while (!rtc.begin())
    {
        #ifdef DEBUG
            Serial.println(F("Couldn't find RTC"));
        #endif
        delay(1000);
    }

    sos.begin();

    logMessage(SF("Start init process..."));

    #ifdef DEBUG
        Serial.println(F("SOS ready"));
        Serial.println(F("Init..."));
    #endif

    // init sensors
    lightMeter.begin();

    #ifdef DEBUG
        Serial.println(F("light ready"));
    #endif

    dht.begin();

    #ifdef DEBUG
        Serial.println(F("DHT ready"));
    #endif

    if (!bme.begin(BME_I2C_ADDR))
    {
        BME_I2C_ADDR = 0x77;
        if(!bme.begin(BME_I2C_ADDR))
        {
            #ifdef DEBUG
                Serial.println(F("BME280 sensor not found"));
            #endif
            logMessage(SF("BME280 sensor not found"));
            delay(10000);
            setup();
        }

    }

    #ifdef DEBUG
        Serial.println(F("BME ready"));
    #endif

    // set pin mode
    pinMode(RAIN, INPUT_PULLUP);
    pinMode(SOIL_A_PIN, INPUT);

    // attach external interrupt pins to IRQ functions (rain and wind speed)
    attachInterrupt(digitalPinToInterrupt(RAIN), rainIRQ, FALLING); // FALLING
    // attachInterrupt(0, rainIRQ, HIGH);
    #ifdef WIND
        pinMode(WSPEED, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(WSPEED), wspeedIRQ, FALLING);
    #endif
    // attachInterrupt(1, wspeedIRQ, HIGH);

    // turn on interrupts
    interrupts();

    #ifdef DEBUG
        Serial.println(F("done"));
    #endif

    logMessage(SF("init process success"));
    delay(2000);

    bool flag = false;

    while(!flag)
    {
        logMessage(SF("Start rtc sync process"));
        flag = syncRTC(com, rtc);
    }

    #ifdef DEBUG
        Serial.println(F("RTC sync success"));
    #endif
    logMessage(SF("start loop"));

    DateTime now = rtc.now();

    uint8_t min = now.minute();

    lastDay = now.day();
    lastMisMin = min;
    lastLogDate = now;

    lastSendDate = now;
    lastSamplingDate = now;

    // #ifdef DEBUG
    //     Serial.print(F("Free RAM: "));
    //     Serial.println(freeRam());
    // #endif

    delay(2000);
}

/**
 * Read soil humidity (0-100)
 *
 * @return short soil humidity
 */
short readSoil() // or float
{
    short value = analogRead(SOIL_A_PIN);
    short measure = ((1023.0 - value) / 1023.0) * 100.0; // or float
    return measure;
}

/**
 *
 * Read mesure from sensors and check data quality
 *
 */
void getWeatherMeasure()
{
    dstemp.requestTemperatures();

    float intTemp = dht.readTemperature();
    delay(150);
    float intHum = dht.readHumidity();

    lightMeter.configure(BH1750_ONE_TIME_HIGH_RES_MODE);
    delay(150);

    uint16_t lux = lightMeter.readLightLevel();
    // istsos max value = 9999
    if (lux >= 10000)
    {
        lux = 9999;
    }

    float pressure = bme.readPressure() / 100.0F;
    float humidity = bme.readHumidity();
    float temp = dstemp.getTempCByIndex(0);
    uint8_t soil = readSoil();

    rain += lastrain;
    lastrain = 0;


    measTemp.addMeasure(temp);
    measHum.addMeasure(humidity);
    measPres.addMeasure(pressure);
    measLux.addMeasure(lux);
    measSoil.addMeasure(soil);
    measIntTemp.addMeasure(intTemp);
    measIntHum.addMeasure(intHum);

    #ifdef WIND
        int winddir = get_wind_direction();
        float windspeedms = get_wind_speed();
        medianWinDir.add(winddir);
        medianWinSp.add(windspeedms);
    #endif
}

/**
 * This function generate a String with the date and the measures
 *
 * @return String string containing date and measures
 */
String formatWeatherMeasure(const DateTime& now) {

    // get current date
    String message = getFormattedDate(now);

    // concat measures
    message += "," + measIntTemp.getAverageQI();
    message += "," + measSoil.getAverageQI();
    message += "," + measLux.getAverageQI();
    message += "," + measPres.getAverageQI();
    message += "," + measHum.getAverageQI();
    message += "," + measTemp.getAverageQI();
    message += "," + String(rain) + SF(":100");
    #ifdef WIND
        message += "," + String(SampMedianWinDir.getAverage()) + SF(":100");
        message += "," + String(SampMedianWinSp.getAverage()) + SF(":100");
    #else
        message += "," + SF("-999:400");
        message += "," + SF("-999:400");
    #endif
    message += "," + measTemp.getAverageQI();

    #ifdef WIND
        SampMedianWinDir.clear();
        SampMedianWinSp.clear();
    #endif

    rain = 0;

    return message;

}

/**
 * Every minute
*/
void medianLastMin()
{

    measTemp.calcLastMin();
    measHum.calcLastMin();
    measPres.calcLastMin();
    measLux.calcLastMin();
    measSoil.calcLastMin();
    measIntTemp.calcLastMin();
    measIntHum.calcLastMin();

    #ifdef WIND
        SampMedianWinDir.add(medianWinDir.getAverage());
        SampMedianWinSp.add(medianWinSp.getAverage());

        medianWinSp.clear();
        medianWinDir.clear();
    #endif
}

// void sendData()
// {
//     uint8_t res = sos.sendData();
//
//     if (res == 0)
//     {
//         sendStatus = true;
//         countSend = 0;
//     }
//
//     countSend++;
//     sendStatus = false;
//     if (countSend >= 3)
//     {
//         sendStatus = true;
//         countSend = 0;
//         return res;
//     }
//
//     return res;
// }

void loop() {

    // check if it's time to log data
    DateTime now = rtc.now();
    uint8_t min = now.minute();

    if(calcSamplingTime(now, lastSamplingDate, 10))
    {
        getWeatherMeasure();
        lastSamplingDate = now;
    }


    if(calcInterval(min, lastMisMin, 1) && min < 60 && now.year() < 2060)
    {
        lastMisMin = min;
        medianLastMin();
        // #ifdef DEBUG
        //     Serial.print(F("Free RAM after minute: "));
        //     Serial.println(freeRam());
        // #endif
    }

    if(calcLogInterval(now, lastLogDate, SAMPLING_TIME_MIN) && min < 60 && now.year() < 2060)
    {
        #ifdef DEBUG
            Serial.println(F("time to log measures"));
        #endif

        String message = formatWeatherMeasure(now);
        #ifdef DEBUG
            Serial.println(message);
        #endif
        lastLogDate = now;

        // temporary removed
        sos.logData(message);

        // #ifdef DEBUG
        //     Serial.print(F("Free RAM after sd log: "));
        //     Serial.println(freeRam());
        // #endif

        if(calcSendTime(now, lastSendDate, SENDING_TIME_MIN) || !sendStatus)
        {
            #ifdef DEBUG
                Serial.print(F(" Sending data: "));
            #endif
            uint8_t res = sendData(sos);
            sendStatus = true;

            #ifdef DEBUG
                Serial.println(res);
            #endif

            lastSendDate = now;

            if(sendStatus && res == 0)
            {
                logMessage(SF("Sending data success"));
            }
            else if(sendStatus && res != 0)
            {
                String logMsg = SF("Sending data failure: ") + String(res);
                logMessage(logMsg);
            }
        }

        uint8_t dayNow = now.day();
        // dayNow < 32 avoid rtc bad read
        if( dayNow != lastDay && sendStatus && dayNow < 32)
        {
            bool flag = syncRTC(com, rtc);

            if(!flag)
            {
                #ifdef DEBUG
                    Serial.println(F("Problem sync RTC..."));
                #endif
                logMessage(SF("Problem RTC sync"));
            }
            else
            {
                do
                {
                    now = rtc.now();
                    lastDay = now.day();
                } while (lastDay > 31);

                #ifdef DEBUG
                    Serial.println(F("RTC sync ok"));
                #endif
                logMessage(SF("RTC sync ok"));
            }
        }

    }
    // back to sleep :)
    delay(4000);

}
