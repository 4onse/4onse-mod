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
// istsos comunication library (GPRS)
#include <istsos.h>
// #include <com/drok.h>
#include <com/sim800.h>
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

// Power saving
#include <avr/power.h>
// #include <LowPower.h>
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
// Drok com = Drok(Serial1, APN, APNUSER, PASS, BASIC_AUTH, SIM_PIN);
Sim800 com = Sim800(Serial1, APN, APNUSER, PASS, BASIC_AUTH, SIM_PIN);
OpenLog sdLog = OpenLog(Serial2);
Istsos sos(sdLog, com, SERVER, URI, PROCEDURE_ID);

/******************************************
 * Global variable FOR INTERRUPTS
 *****************************************/

volatile float lastrain = 0;
float rain = 0.0;
uint8_t windClicks = 0;
volatile unsigned long lastWindCheck = 0;
volatile unsigned long lastWindIRQ = 0;
volatile unsigned long rainlast;

/******************************************
 * Arrays to get the median values last minute
 *****************************************/
RunningMedian medianTemp    = RunningMedian(SAMPLING_TIME_MED);
RunningMedian medianHum     = RunningMedian(SAMPLING_TIME_MED);
RunningMedian medianPres    = RunningMedian(SAMPLING_TIME_MED);
RunningMedian medianSoil    = RunningMedian(SAMPLING_TIME_MED);
RunningMedian medianLux     = RunningMedian(SAMPLING_TIME_MED);
RunningMedian medianIntTemp = RunningMedian(SAMPLING_TIME_MED);
RunningMedian medianIntHum  = RunningMedian(SAMPLING_TIME_MED);
RunningMedian medianWinDir  = RunningMedian(SAMPLING_TIME_MED);
RunningMedian medianWinSp   = RunningMedian(SAMPLING_TIME_MED);

/******************************************
 * Arrays to get the median values last minute
 *****************************************/
RunningMedian SampMedianTemp    = RunningMedian(SAMPLING_TIME_MEDIAN);
RunningMedian SampMedianHum     = RunningMedian(SAMPLING_TIME_MEDIAN);
RunningMedian SampMedianPres    = RunningMedian(SAMPLING_TIME_MEDIAN);
RunningMedian SampMedianSoil    = RunningMedian(SAMPLING_TIME_MEDIAN);
RunningMedian SampMedianLux     = RunningMedian(SAMPLING_TIME_MEDIAN);
RunningMedian SampMedianIntTemp = RunningMedian(SAMPLING_TIME_MEDIAN);
RunningMedian SampMedianIntHum  = RunningMedian(SAMPLING_TIME_MEDIAN);
RunningMedian SampMedianWinDir  = RunningMedian(SAMPLING_TIME_MEDIAN);
RunningMedian SampMedianWinSp   = RunningMedian(SAMPLING_TIME_MEDIAN);

// Variable to manage data log and data send
uint8_t lastMin = 0;
uint8_t lastDay = 0;
DateTime lastLogDate;
uint8_t lastMisMin = 0;
DateTime lastSendDate;
DateTime lastSamplingDate;

uint8_t countSend = 0;
bool sendStatus = true;


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
        lastrain += 0.2; //0.011;
        rainlast = millis(); // set up for next event
    }
}

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

/**
*   Log message inside SD
*   @param message
*/
void logMessage(const String message)
{
    String date = getFormattedDate(rtc.now());
    sos.logging(date, message);

    Serial.print(date + " : ");
    Serial.println(message);

}

/**
    Initialization
*/
void setup() {

    // disable unuset elements
    power_spi_disable();

    // init serial port
    Serial.begin(9600);
    while(!Serial){}
    Serial1.begin(57200);
    while(!Serial1){}
    Serial2.begin(9600);
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

    logMessage(SF("Start init process..."));

    sos.begin();

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
    pinMode(WSPEED, INPUT_PULLUP);
    pinMode(RAIN, INPUT_PULLUP);
    pinMode(SOIL_A_PIN, INPUT);

    // attach external interrupt pins to IRQ functions (rain and wind speed)
    attachInterrupt(digitalPinToInterrupt(RAIN), rainIRQ, FALLING); // FALLING
    // attachInterrupt(0, rainIRQ, HIGH);
    attachInterrupt(digitalPinToInterrupt(WSPEED), wspeedIRQ, FALLING);
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

    Serial.println(F("RTC sync success"));
    logMessage(SF("start loop"));

    DateTime now = rtc.now();

    uint8_t min = now.minute();

    lastDay = now.day();
    lastMisMin = min;
    lastLogDate = now;

    lastSendDate = now;
    lastSamplingDate = now;

    delay(2000);
}

/**
 * Read soil humidity (0-100)
 *
 * @return short soil humidity
 */
short readSoil()
{
    short value = analogRead(SOIL_A_PIN);
    short measure = ((1023.0 - value) / 1023.0) * 100.0;
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

    int winddir = get_wind_direction();
    float windspeedms = get_wind_speed();

    float intTemp = dht.readTemperature();
    float intHum = dht.readHumidity();

    rain += lastrain;

    lastrain = 0;

    if(temp >= -80.0 && temp <= 60.0)
    {
        checkMinVar(medianTemp, temp, 2.0);
    }
    if(humidity >= 0.0 && humidity <= 100.0)
    {
        checkMinVar(medianHum, humidity, 5.0);
    }
    if(pressure >= 500.0 && pressure <= 1100.0)
    {
        checkMinVar(medianPres, pressure, 0.3);
    }
    if(lux >= 0.0 && lux <= 100000)
    {
        medianLux.add(lux);
    }
    if(soil >= 0.0 && soil <= 100.0)
    {
        medianSoil.add(soil);
    }

    if(intTemp >= -60.0 && intTemp <= 100.0)
    {
        checkMinVar(medianIntTemp, intTemp, 3.0);
        // medianIntTemp.add(intTemp);
    }
    if(intHum >= 0.0 && intHum <= 100.0)
    {
        checkMinVar(medianIntHum, intHum, 10.0);
        // medianIntHum.add(intHum);
    }

    medianWinDir.add(winddir);
    medianWinSp.add(windspeedms);
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
    message += "," + String(SampMedianIntTemp.getAverage());
    message += "," + String(SampMedianSoil.getAverage());
    message += "," + String(SampMedianLux.getAverage());
    message += "," + String(SampMedianPres.getAverage());
    message += "," + String(SampMedianHum.getAverage());
    message += "," + String(SampMedianTemp.getAverage());
    message += "," + String(rain);
    message += "," + String(SampMedianWinDir.getAverage());
    message += "," + String(SampMedianWinSp.getAverage());
    message += "," + String(SampMedianIntHum.getAverage());

    // empty median arrays
    SampMedianTemp.clear();
    SampMedianHum.clear();
    SampMedianPres.clear();
    SampMedianSoil.clear();
    SampMedianLux.clear();
    SampMedianIntTemp.clear();
    SampMedianIntHum.clear();
    SampMedianWinDir.clear();
    SampMedianWinSp.clear();

    rain = 0;

    return message;

}

/**
 * Every minute
*/
void medianLastMin()
{

    checkVar(SampMedianTemp, medianTemp, 2.0);
    checkVar(SampMedianHum, medianHum, 10.0);
    checkVar(SampMedianPres, medianPres, 0.5);

    checkVar(SampMedianIntTemp, medianIntTemp, 5.0);
    checkVar(SampMedianIntHum, medianIntHum, 20.0);

    SampMedianLux.add(medianLux.getAverage());
    SampMedianSoil.add(medianSoil.getAverage());
    SampMedianWinDir.add(medianWinDir.getAverage());
    SampMedianWinSp.add(medianWinSp.getAverage());

    medianLux.clear();
    medianSoil.clear();
    medianWinSp.clear();
    medianWinDir.clear();
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
    }

    if(calcLogInterval(now, lastLogDate, SAMPLING_TIME_MIN) && min < 60 && now.year() < 2060)
    {
        Serial.println(F("time to log measures"));

        String message = formatWeatherMeasure(now);
        Serial.println(message);
        lastLogDate = now;

        sos.logData(message);

        if(calcSendTime(now, lastSendDate, SENDING_TIME_MIN) || !sendStatus)
        {
            Serial.print(F(" Sending data: "));
            bool res = sendData(sos);
            Serial.println(res);
            lastSendDate = now;

            if(res)
            {
                logMessage(SF("Sending data success"));
            }
            else
            {
                logMessage(SF("Sending data failure"));
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
