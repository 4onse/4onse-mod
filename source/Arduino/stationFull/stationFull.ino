#define DEBUG
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

// Power saving
// #include <LowPower.h>
#include "functions.h"

#include <avr/power.h>



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

// BME could be 0x76 or 0x77
uint8_t BME_I2C_ADDR = 0x76;

#define DHTPIN 10
#define DHTTYPE DHT11
#define FAN_PIN 12

#define WSPEED 3
#define RAIN 2
#define WDIR A0

#define SAMPLING_TIME_MIN 5
#define SENDING_TIME_MIN 15

#define SF(x) String(F(x))

#define MEDIAN_LENGTH SAMPLING_TIME_MIN

DHT dht(DHTPIN, DHTTYPE);

/*****************************************
 * Sensors definition
 ****************************************/
BH1750 lightMeter;
Adafruit_BME280 bme;
RTC_DS3231 rtc;
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature dstemp(&oneWire);

/*****************************************
 * Comunication and logging system
 ****************************************/
Drok com = Drok(Serial1, APN, APNUSER, PASS, BASIC_AUTH, SIM_PIN, false);
OpenLog sdLog = OpenLog(Serial2);
Istsos sos(sdLog, com, SERVER, URI, PROCEDURE_ID);

/******************************************
 * Global variable definition
 *****************************************/
// uint16_t lux  = 0;
// float pressure = 0.0;
// float humidity = 0.0;
// float temp = 0.0;
// short soil = 0;
// float intTemp = 0.0;

/******************************************
 * Arrays to get the median values
 *****************************************/
RunningMedian medianTemp    = RunningMedian(MEDIAN_LENGTH);
RunningMedian medianHum     = RunningMedian(MEDIAN_LENGTH);
RunningMedian medianPres    = RunningMedian(MEDIAN_LENGTH);
RunningMedian medianSoil    = RunningMedian(MEDIAN_LENGTH);
RunningMedian medianLux     = RunningMedian(MEDIAN_LENGTH);
RunningMedian medianIntTemp = RunningMedian(MEDIAN_LENGTH);
RunningMedian medianWinDir  = RunningMedian(MEDIAN_LENGTH);

uint8_t lastMin = 0;
uint8_t lastDay = 0;
uint8_t lastLogMin = 0;
uint8_t lastMisMin = 0;
DateTime lastSendDate;

//These are all the weather values that wunderground expects:
short winddir = 0; // [0-360 instantaneous wind direction]
float windspeedms = 0; // [m/s instantaneous wind speed]
volatile float lastrain = 0;
float rain = 0.0;

// char timezone[7];

// wind and rain variable
uint8_t windClicks = 0;
volatile unsigned long lastWindCheck = 0;
volatile long lastWindIRQ = 0;
volatile unsigned long raintime, rainlast;
// unsigned long lastRun, lastSend;

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Hardware interrupt
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

/*
   Count rain gauge bucket tips as they occur
   ctivated by the magnet and reed switch in the rain gauge, attached to input D2
*/
void rainIRQ()
{
    // lastrain += 0.2; //0.011;
    // return;
    raintime = millis(); // grab current time

    if ((unsigned long)(millis() - rainlast) > 10) // ignore switch-bounce glitches less than 10mS after initial edge
    {
        lastrain += 0.2; //0.011;
        rainlast = millis(); // raintime; // set up for next event
    }
}

/*
   Activated by the magnet in the anemometer (2 ticks per rotation), attached to input D3
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
/*
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

float getInternalTemp()
{
    return dht.readTemperature();
}

/**
 * Check the internal temp and start the fan if it's above 30°
 */
void checkInternalTemp()
{
    // float t = getInternalTemp();

    // remove return to start the fan
    return;

    // if (t >= 40.0)
    // {
    //     Serial.println(F("Start fan...."));
    //     digitalWrite(FAN_PIN, LOW);
    //     delay(20000);
    //     Serial.println(F("Stop fan..."));
    //     digitalWrite(FAN_PIN, HIGH);
    // }
}


void logMessage(const String message)
{
    String date = getFormattedDate(rtc.now());
    sos.logging(date, message);
}

void setup() {

    // disable unuset elements
    power_spi_disable();
    // power_usart3_disable();
        // init serial port

    Serial.begin(9600);
    while(!Serial){}
    Serial1.begin(57200);
    while(!Serial1){}
    Serial2.begin(9600);
    while(!Serial2){}
    delay(3000);


    if (!rtc.begin())
    {
        #ifdef DEBUG
            Serial.println(F("Couldn't find RTC"));
        #endif
        return;
    }

    sos.begin();

    #ifdef DEBUG
        Serial.println(F("SOS ready"));
        Serial.println(F("Init..."));
    #endif

    logMessage(SF("Start init process..."));

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



    pinMode(WSPEED, INPUT_PULLUP);
    pinMode(RAIN, INPUT_PULLUP);
    pinMode(FAN_PIN, OUTPUT);
    digitalWrite(FAN_PIN, HIGH);
    // set pin mode
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

    logMessage(SF("RTC sync success"));

    // checkInternalTemp();

    logMessage(SF("start loop"));

    delay(2000);

    DateTime now = rtc.now();

    uint8_t min = now.minute();

    lastDay = now.day();
    lastMisMin = min;
    lastLogMin = min;

    lastSendDate = now;

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
 * Read mesure from sensrs
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

    winddir = get_wind_direction();
    windspeedms = get_wind_speed();

    float intTemp = getInternalTemp();

    rain += lastrain;

    lastrain = 0;

    medianTemp.add(temp);
    medianHum.add(humidity);
    medianPres.add(pressure);
    medianSoil.add(soil);
    medianLux.add(lux);
    medianIntTemp.add(intTemp);
    medianWinDir.add(winddir);

}

/**
 * This function generate a String with the date and the measures
 *
 * @return String string containing date and measures
 */
String formatWeatherMeasure(const DateTime& now) {

    String message = getFormattedDate(now);

    message += "," + String(medianIntTemp.getMedian());
    message += "," + String(medianSoil.getMedian());
    message += "," + String(medianLux.getMedian());
    message += "," + String(medianPres.getMedian());
    message += "," + String(medianHum.getMedian());
    message += "," + String(medianTemp.getMedian());
    message += "," + String(rain);
    message += "," + String(medianWinDir.getMedian());
    message += "," + String(windspeedms);

    medianTemp.clear();
    medianHum.clear();
    medianPres.clear();
    medianSoil.clear();
    medianLux.clear();
    medianIntTemp.clear();
    medianWinDir.clear();

    rain = 0;

    return message;

}

uint8_t count = 0;
bool sendStatus = true;
/**
    Send data to the server
*/
void sendData()
{
    // String date = getFormattedDate(now);
    logMessage(SF("Sending data..."));

    bool res = sos.sendData();

    if (res)
    {
        logMessage(SF("data send"));
        Serial.println(F("data send"));
        sendStatus = true;
        count = 0;
    }
    else
    {
        count++;
        sendStatus = false;
        if (count >= 3)
        {
            logMessage(SF("problem sending data"));
            Serial.println(F("problem sending data"));
            sendStatus = true;
            count = 0;
        }
    }
}

void loop() {

    // check if it's time to log data
    DateTime now = rtc.now();
    uint8_t min = now.minute();

    // check if it's time to read measures
    if(calcInterval(min, lastMisMin, 1) && min < 60 && now.year() < 2060)
    {

        lastMisMin = min;

        getWeatherMeasure();
        // Serial.println(F("Get measures"));

        // check if it's time to log data
        if(calcInterval(min, lastLogMin, SAMPLING_TIME_MIN) && min < 60 && now.year() < 2060)
        {
            String message = formatWeatherMeasure(now);
            Serial.println(message);
            sos.logData(message);
            lastLogMin = now.minute();
        }


        // check if it's time to send data
        if(calcSendTime(now, lastSendDate, SENDING_TIME_MIN) || !sendStatus)
        {
            #ifdef DEBUG
                Serial.println(F("Time to send data"));
            #endif
            sendData();

            if (count == 0)
            {
                lastSendDate = now;
            }
            delay(1000);
        }

        // sync RTC every day
        uint8_t dayNow = now.day();
        // dayNow < 32 avoid rtc bad read
        if( dayNow != lastDay && sendStatus && dayNow < 32)
        {
            bool flag = syncRTC(com, rtc);

            if(!flag)
            {
                Serial.println(F("Problem sync RTC..."));
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
            }
        }
    }
    // back to sleep :)
    delay(8000);

}
