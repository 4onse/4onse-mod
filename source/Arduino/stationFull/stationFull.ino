// =========================================================================
//
// Copyright (C) 2012-2018, Istituto Scienze della Terra
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

#define LCD // for PCB version. Comment it if LCD is not present

// istsos comunication library (GPRS)
#include <istsos.h>
#include <com/drok.h> // <com/drok.h> or <com/sim800.h>
#include <log/sdOpenlog.h>

// temperature
#include <OneWire.h>
#include <DallasTemperature.h>

// pressure/humidity
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

// light
#include <BH1750.h>
#include <Wire.h>
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
#define SERVER "istsos.org"
#define URI "/istsos/wa/istsos/services/demo/operations/fastinsert"
#define PROCEDURE_ID ""
#define BASIC_AUTH ""
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
#define SENDING_TIME_MIN 5     // Sending time (minutes)

// how many measures every minutes
#define SAMPLING_TIME_MED 6
//
#define SAMPLING_TIME_MEDIAN SAMPLING_TIME_MIN


/*****************************************
 * Sensors definition
 ****************************************/
DHT dht(DHTPIN, DHTTYPE);
BH1750 lightMeter(0x23);
Adafruit_BME280 bme;
RTC_DS3231 rtc;
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature dstemp(&oneWire);


/*****************************************
 * Comunication and logging system
 ****************************************/
// Sim800 com = Sim800(Serial1, APN, APNUSER, PASS, BASIC_AUTH, SIM_PIN);
Drok com = Drok(Serial1, APN, APNUSER, PASS, BASIC_AUTH, SIM_PIN);
OpenLog sdLog = OpenLog(Serial2);
Istsos sos(sdLog, com, SERVER, URI, PROCEDURE_ID);

#ifdef LCD
  // include the library code:
  #include <LiquidCrystal.h>
  #define LCD_PIN 7
  // initialize the library by associating any needed LCD interface pin
  // with the arduino pin number it is connected to
  const int rs = 41, en = 39, d4 = 37, d5 = 35, d6 = 33, d7 = 31;
  LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
#endif

#define AWAKE_PIN 6

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
//    RunningMedian SampMedianWinDir  = RunningMedian(SAMPLING_TIME_MEDIAN);
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
uint8_t year_current;


uint8_t countSend = 0;
bool sendStatus = true;
uint8_t countRetry = 0;

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
        if (adc < 890) return (315);
        if (adc < 950) return (270);
        return -1;
    }
#endif

/**
*   Log message inside SD
*   @param message
*/
void logMessage(const String message, bool rtcdate = 1)
{
    if (rtcdate) {
        String date = getFormattedDate(rtc.now());
        sos.logging(date, message);

        #ifdef DEBUG
            Serial.print(date + " : ");
            Serial.println(message);
            #ifdef LCD
              lcd.clear();
              lcd.print(message);
            #endif
        #endif
    } else {
        Serial.println(message);
        #ifdef LCD
          lcd.clear();
          lcd.print(message);
        #endif
    }
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

    delay(150);
    float lux;
    float humidity;
    float pressure;
    
    // read int temp
    float intTemp = dht.readTemperature();
    if (isnan(intTemp))
    {
        intTemp = -999.99;
    }

    // read soil
    uint8_t soil = readSoil();

    // read BH1750
    if (lightMeter.begin(BH1750::ONE_TIME_HIGH_RES_MODE_2)) {
        lux = lightMeter.readLightLevel();
    } else {
        lux = -999.99;
    }

    // read BME280
    if (!bme.begin(BME_I2C_ADDR))
    {
        BME_I2C_ADDR = 0x77;
        delay(500);
        if (!bme.begin(BME_I2C_ADDR))
        {
            
            BME_I2C_ADDR = 0x76;
            delay(500);
            pressure = -999.99;
            humidity = -999.99;
        }
        else
        {
            delay(150);
            pressure = bme.readPressure() / 100.0F;
            humidity = bme.readHumidity();
        }
    } else {
        delay(150);
        pressure = bme.readPressure() / 100.0F;
        humidity = bme.readHumidity();
    }
    if (isnan(pressure))
    {
        pressure = -999.99;
    }

    if (isnan(humidity))
    {
        pressure = -999.99;
    }

    // read DS18B20
    float temp = dstemp.getTempCByIndex(0);
    if (temp == -127.00)
    {
        temp = -999.99;
    }

//    #ifdef WIND
//        // read wind dir
//        float winddir = get_wind_direction();
//        if (winddir < 0)
//        {
//            winddir = -999.99;
//        }
//    #endif

    // read wind speed
    float windspeedms = get_wind_speed();

    // read rain
    rain += lastrain;
    lastrain = 0;


    measTemp.addMeasure(temp);
    measHum.addMeasure(humidity);
    measPres.addMeasure(pressure);
    measLux.addMeasure(lux);
    measSoil.addMeasure(soil);
    measIntTemp.addMeasure(intTemp);

    #ifdef WIND
//        medianWinDir.add(winddir);
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
    message += "," + String(rain);
    #ifdef WIND
        int winddir = get_wind_direction();
        if (winddir<0)
        {
            message += ",-999.99" + SF(":400");    
        } else if (isnan(winddir))
        {
            message += ",-999.99" + SF(":400");    
        } else {
            message += "," + String(winddir);
        }
        float windSpeed = SampMedianWinSp.getAverage();
        if (isnan(winddir))
        {
            message += ",-999.99" + SF(":400");    
        } else {
            message += "," + String(windSpeed);
        }
    #else
        message += "," + SF("-999:400");
        message += "," + SF("-999:400");
    #endif

    #ifdef WIND
//        SampMedianWinDir.clear();
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
//        SampMedianWinDir.add(medianWinDir.getAverage());
        SampMedianWinSp.add(medianWinSp.getMedian());

        medianWinSp.clear();
        medianWinDir.clear();
    #endif
}

/**
    Initialization
*/
void setup() {

    // clock_prescale_set(clock_div_2);
    #ifdef LCD
      // set up the LCD's number of columns and rows:
      lcd.begin(16, 2);
      pinMode(LCD_PIN, OUTPUT);
    #endif

    pinMode(LED_BUILTIN, OUTPUT);    
    digitalWrite(LED_BUILTIN, LOW);

    pinMode(AWAKE_PIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);    


    // init serial port
    Serial.begin(9600);     // Serial to PC
    while(!Serial){}
    Serial1.begin(57200);   // Serial to GPRS
    while(!Serial1){}
    Serial2.begin(9600);    // Serial to SD
    while(!Serial2){}
    delay(500);

    Serial.println("++++Start init process++++");
    #ifdef LCD
      lcd.print("Init");
    #endif

    while(!sos.beginSD())
    {
        Serial.println("Openlog not found");
        #ifdef LCD
          lcd.clear();
          lcd.print("ERROR Openlog");
        #endif
        delay(1000);
        alert(1);
    }
    logMessage(SF("SD ready"), 0);

    while(!sos.beginSIM800())
    {
        logMessage(SF("ERROR SIM800"), 0);
        delay(1000);
        alert(2);
    }

    logMessage(SF("SIM800 ready"), 0);

    Wire.begin();
    // disable unuset elements
    logMessage(SF("Optimizing power"), 0);
    power_spi_disable();

    // this variables are used in function.cpp
    countSend = 0;
    sendStatus = true;
    byte error;

    // check rtc
    while(1)
    {
        Wire.beginTransmission(104);
        error = Wire.endTransmission();
        if (error == 0)
        {
            break;
        } else {
            logMessage(SF("ERROR RTC 0"), 0);
            alert(3);
        }
        delay(5000);
    }
    while (!rtc.begin())
    {
        logMessage(SF("ERROR RTC 1"), 0);
        delay(1000);
        alert(3);
    }
    logMessage(SF("RTC ready"), 0);
    
    logMessage(SF("Init sensors"));

    // init sensors

    // check BH1750
    while(countRetry<=10)
    {
        if (lightMeter.begin(BH1750::ONE_TIME_HIGH_RES_MODE_2)) {
            logMessage(SF("BH1750 ready"));
            break;
        }
        else {
            logMessage(SF("ERROR BH1750"));
        }
        countRetry++;
    }
    if (countRetry >= 10)
    {
        bool flagBH = false;
        alert(4);
    } else {
        bool flagBH = true;
    }
    countRetry = 0;

    // check DHT
    dht.begin();

    logMessage(SF("DHT ready"));

    // check BME280
    while(countRetry<=10)
    {
        if (!bme.begin(BME_I2C_ADDR))
        {
            BME_I2C_ADDR = 0x77;
            delay(500);
            if (!bme.begin(BME_I2C_ADDR))
            {
                logMessage(SF("ERROR BME280"));
                BME_I2C_ADDR = 0x76;
                delay(500);
            }
            else
            {
                logMessage(SF("BME ready"));
                break;
            }
        }
        else
        {
            logMessage(SF("BME ready"));
            break;
        }
        countRetry++;
    }
    if (countRetry >= 10)
    {
        bool flagBME = false;
        alert(5);
    } else {
        bool flagBME = true;
    }
    countRetry = 0;

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

    logMessage(SF("Done"));

    logMessage(SF("Init ended"));
    delay(2000);

    bool flag = false;

    digitalWrite(LED_BUILTIN, HIGH);
    while(!flag)
    {
        logMessage(SF("RTC syncing"));
        flag = syncRTC(com, rtc);
    }
    year_current = rtc.now().year();
    digitalWrite(LED_BUILTIN, LOW);

    #ifdef DEBUG
        logMessage(SF("RTC sync success"));
    #endif

    if (sos.checkMissingData())
    {
        while(countRetry<5)
        {
            uint8_t flag = sos.sendData();

            if(flag == REQUEST_SUCCESS)
            {
                logMessage(SF("Missing data sent"));
                break;
            } else {
                alert_long(2);
            }
            delay(500);
            countRetry++;
        }
    }
    else
    {
        Wire.beginTransmission(104);
        if (Wire.endTransmission() != 0)
        {
            setup();
        }

        String tmpDate = getFormattedDate(rtc.now());
        String tmp = SF(PROCEDURE_ID) + SF(";") + tmpDate;

        dstemp.begin();
        dstemp.requestTemperatures();

        delay(150);
        float lux;
        float humidity;
        float pressure;
        
        // read int temp
        float intTemp = dht.readTemperature();
        if (isnan(intTemp))
        {
            intTemp = -999.99;
            tmp += SF(",") + String(intTemp) + SF(":400");
        } else {
            tmp += SF(",") + String(intTemp) + SF(":800");
        }

        // read soil
        uint8_t soil = readSoil();
        tmp += SF(",") + String(soil) + SF(":800");

        // read BH1750
        if (lightMeter.begin(BH1750::ONE_TIME_HIGH_RES_MODE_2)) {
            lux = lightMeter.readLightLevel();
            tmp += SF(",") + String(lux) + SF(":800");
        } else {
            lux = -999.99;
            tmp += SF(",") + String(lux) + SF(":400");
        }

        // read BME280
        if (!bme.begin(BME_I2C_ADDR))
        {
            BME_I2C_ADDR = 0x77;
            delay(500);
            if (!bme.begin(BME_I2C_ADDR))
            {
                
                BME_I2C_ADDR = 0x76;
                delay(500);
                pressure = -999.99;
                humidity = -999.99;
            }
            else
            {
                delay(150);
                pressure = bme.readPressure() / 100.0F;
                humidity = bme.readHumidity();
            }
        } else {
            delay(150);
            pressure = bme.readPressure() / 100.0F;
            humidity = bme.readHumidity();
        }
        if (isnan(pressure) || pressure == -999.99)
        {
            pressure = -999.99;
            tmp += SF(",") + String(pressure) + SF(":400");
        } else {
            tmp += SF(",") + String(pressure) + SF(":800");
        }

        if (isnan(humidity) || pressure == -999.99)
        {
            pressure = -999.99;
            tmp += SF(",") + String(humidity) + SF(":400");
        } else {
            tmp += SF(",") + String(humidity) + SF(":800");
        }

        // read DS18B20
        float temp = dstemp.getTempCByIndex(0);
        if (temp == -127.00)
        {
            temp = -999.99;
            tmp += SF(",") + String(temp) + SF(":400");
        } else {
            tmp += SF(",") + String(temp) + SF(":800");
        }

        // read rain
        tmp += SF(",") + String(rain) + SF(":800");

        // read wind dir
        int winddir = get_wind_direction();
        if (winddir < 0)
        {
          tmp += SF(",") + String(-999.99) + SF(":400");
        } else if (isnan(winddir))
        {
          tmp += SF(",") + String(-999.99) + SF(":400");
        } else {
          tmp += SF(",") + String(winddir) + SF(":800");
        }

        // read wind speed
        float windspeedms = get_wind_speed();
        tmp += SF(",") + String(windspeedms) + SF(":800");

        #ifdef DEBUG
            Serial.print(F("Test string: "));
            Serial.println(tmp);
            #ifdef LCD
              lcd.clear();
              lcd.print("Test com string");
            #endif
        #endif

        while(countRetry<5)
        {
            uint8_t code = com.executePost(SERVER, URI, tmp);

            #ifdef DEBUG
                Serial.print(F("Error code: "));
                Serial.println(code);
                #ifdef LCD
                  lcd.clear();
                  lcd.print("ERROR CODE: ");
                  lcd.print(code);
                #endif
            #endif

            if(code == 0)
            {
                break;
            } else {
                alert_long(2);
            }
            countRetry++;
            delay(500);
        }
    }
    countRetry = 0;


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
    
    logMessage(SF("Start loop"));

    delay(2000);

    digitalWrite(LED_BUILTIN, LOW);

}

void loop() {

    digitalWrite(AWAKE_PIN, HIGH);
    delay(50);
    digitalWrite(AWAKE_PIN, LOW);

    // check if it's time to log data
    DateTime now = rtc.now();
    uint8_t min = now.minute();
    while(1)
    {
        Wire.beginTransmission(104);
        if (Wire.endTransmission() == 0)
        {
            break;
        } else {
            setup();
        }
        delay(5000);
    }
    uint8_t year = now.year();
    if (year != year_current)
    {
        setup();
    }

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

        if(calcSendTime(now, lastSendDate, SENDING_TIME_MIN))
        {
            while(countSend<3)
            {
                #ifdef DEBUG
                    Serial.print(F(" Sending data: "));
                #endif
                uint8_t res = sos.sendData();
                // sendStatus = true;
                #ifdef DEBUG_RAM
                    Serial.print(F("Free RAM after sd log: "));
                    Serial.println(freeRam());
                #endif

                #ifdef DEBUG
                    Serial.println(res);
                #endif

                lastSendDate = now;

                if(res == 0)
                {
                    logMessage(SF("Data sent"));
                    digitalWrite(LED_BUILTIN, LOW);
                    break;
                }
                countSend++;
                if (countSend==3)
                {
                    String logMsg = SF("ERROR CODE ") + String(res);
                    logMessage(logMsg);
                    alert_long(2);
                    digitalWrite(LED_BUILTIN, HIGH);
                }
            }
            countSend = 0;
        }

        uint8_t dayNow = now.day();
        // Sync the rtc once a week
        if(now.dayOfTheWeek() == 0 && dayNow != lastDay && sendStatus)
        {
            bool flag = syncRTC(com, rtc);

            if(!flag)
            {
                logMessage(SF("Problem RTC sync"));
            }
            else
            {
                do
                {
                    now = rtc.now();
                    lastDay = now.day();
                } while (lastDay > 31);

                logMessage(SF("RTC sync ok"));
            }
        }

    }
    // back to sleep :)
    delay(2000);
}
