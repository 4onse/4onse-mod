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
// #define DEBUG_RAM
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

// Power saving
#include <avr/power.h>


/*****************************************
 * Configure params
 ****************************************/
#define SERVER "xxx"
#define URI "/4onse/wa/istsos/services/sos/operations/fastinsert"
#define PROCEDURE_ID "xxx"
#define BASIC_AUTH "xxx"
//
#define APN ""
#define APNUSER ""
#define PASS ""
#define SIM_PIN ""

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

#define SAMPLING_TIME_MIN 5     // Sampling logging time (minutes)
#define SENDING_TIME_MIN 10     // Sending time (minutes)

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
// Measure measIntHum   = Measure(SAMPLING_TIME_MED, SAMPLING_TIME_MEDIAN, 0.0, 100.0, 5.0, 10.0);

/*********************************************
 * Variable to manage data log and data send
 ********************************************/

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
    Initialization
*/
void setup() {

    // disable unuset elements
    power_spi_disable();


    // clock_prescale_set(clock_div_2);

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);


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

    alert(3);
    // check rtc
    while (!rtc.begin())
    {
        #ifdef DEBUG
            Serial.println(F("Couldn't find RTC"));
        #endif
        delay(5000);
    }

    alert(4);
    while(!sos.begin())
    {
        delay(5000);
    }

    #ifdef DEBUG
        Serial.println(F("SOS ready"));
        Serial.println(F("Init..."));
    #endif

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

    alert(5);

    while(1)
    {
        if (!bme.begin(BME_I2C_ADDR))
        {
            BME_I2C_ADDR = 0x77;
            if (!bme.begin(BME_I2C_ADDR))
            {
                #ifdef DEBUG
                    Serial.println(F("BME280 sensor not found"));
                #endif
                logMessage(SF("BME280 sensor not found"));
                BME_I2C_ADDR = 0x76;
                delay(5000);
            }
            else
            {
                break;
            }
        }
        else
        {
            break;
        }
    }


    #ifdef DEBUG
        Serial.println(F("BME ready"));
    #endif

    // set pin mode
    pinMode(RAIN, INPUT_PULLUP);
    pinMode(SOIL_A_PIN, INPUT);

    attachInterrupt(digitalPinToInterrupt(RAIN), rainIRQ, FALLING); // FALLING
    #ifdef WIND
        pinMode(WSPEED, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(WSPEED), wspeedIRQ, FALLING);
    #endif

    // turn on interrupts
    interrupts();

    #ifdef DEBUG
        Serial.println(F("done"));
    #endif

    logMessage(SF("init process success"));
    delay(2000);

    bool flag = false;

    alert(6);
    while(!flag)
    {
        logMessage(SF("Start rtc sync process"));
        flag = syncRTC(com, rtc);
    }

    #ifdef DEBUG
        Serial.println(F("RTC sync success"));
    #endif

    if (sos.checkMissingData())
    {
        while(1)
        {
            uint8_t flag = sos.sendData();

            if(flag == REQUEST_SUCCESS)
            {
                logMessage(SF("Uploaded missing data"));
                break;
            }
            delay(10000);
        }
    }
    else
    {
        String tmpDate = getFormattedDate(rtc.now());
        String tmp = SF(PROCEDURE_ID) + SF(";") + tmpDate;

        dstemp.requestTemperatures();

        lightMeter.configure(BH1750_ONE_TIME_HIGH_RES_MODE);
        delay(150);

        uint16_t lux = lightMeter.readLightLevel();
        float pressure = bme.readPressure() / 100.0F;
        float humidity = bme.readHumidity();
        float temp = dstemp.getTempCByIndex(0);
        uint8_t soil = readSoil();
        int winddir = get_wind_direction();
        float windspeedms = get_wind_speed();
        float intTemp = dht.readTemperature();

        tmp += SF(",") + String(intTemp) + SF(":800");
        tmp += SF(",") + String(soil) + SF(":800");
        tmp += SF(",") + String(lux) + SF(":800");
        tmp += SF(",") + String(pressure) + SF(":800");
        tmp += SF(",") + String(humidity) + SF(":800");
        tmp += SF(",") + String(temp) + SF(":800");
        tmp += SF(",") + String(rain) + SF(":800");
        tmp += SF(",") + String(winddir) + SF(":800");
        tmp += SF(",") + String(windspeedms) + SF(":800");

        #ifdef DEBUG
            Serial.print(F("Test string: "));
            Serial.println(tmp);
        #endif

        alert(7);

        while(1)
        {
            uint8_t code = com.executePost(SERVER, URI, tmp);

            #ifdef DEBUG
                Serial.print(F("Error code: "));
                Serial.println(code);
            #endif

            if(code == 0)
            {
                break;
            }
            delay(5000);
        }
    }

    logMessage(SF("start loop"));

    DateTime now = rtc.now();

    uint8_t min = now.minute();

    lastDay = now.day();
    lastMisMin = min;
    lastLogDate = now;

    lastSendDate = now;
    lastSamplingDate = now;

    #ifdef DEBUG_RAM
        Serial.print(F("Free RAM: "));
        Serial.println(freeRam());
    #endif

    delay(2000);

    digitalWrite(LED_BUILTIN, LOW);

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

    lightMeter.configure(BH1750_ONE_TIME_HIGH_RES_MODE);
    delay(150);

    uint16_t lux = lightMeter.readLightLevel();
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
    // message += "," + measTemp.getAverageQI();

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
    // measIntHum.calcLastMin();

    #ifdef WIND
        SampMedianWinDir.add(medianWinDir.getAverage());
        SampMedianWinSp.add(medianWinSp.getAverage());

        medianWinSp.clear();
        medianWinDir.clear();
    #endif
}

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
        #ifdef DEBUG_RAM
            Serial.print(F("Free RAM after minute: "));
            Serial.println(freeRam());
        #endif
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

        #ifdef DEBUG_RAM
            Serial.print(F("Free RAM after sd log: "));
            Serial.println(freeRam());
        #endif

        if(calcSendTime(now, lastSendDate, SENDING_TIME_MIN) || !sendStatus)
        {
            #ifdef DEBUG
                Serial.print(F(" Sending data: "));
            #endif
            uint8_t res = sendData(sos);
            // sendStatus = true;

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
        // Sync the rtc once a week
        if(now.dayOfTheWeek() == 0 && dayNow != lastDay && sendStatus)
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
