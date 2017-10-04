
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

// Power saving
// #include <LowPower.h>
#include "functions.h"

/*****************************************
 * Configure params
 ****************************************/
// #define SERVER "geoservice.ist.supsi.ch"
// #define URI "/4onse/wa/istsos/services/sos/operations/fastinsert"
// #define PROCEDURE_ID "175378da9a0511e79e2008002745029a"
//
// #define APN "gprs.swisscom.ch"
// #define APNUSER "gprs"
// #define PASS "gprs"


/*****************************************
 * Define sensor pin
 ****************************************/
#define ONE_WIRE_BUS 9
#define SOIL_A_PIN A13

// #define BME_I2C_ADDR 0x77
uint8_t BME_I2C_ADDR = 0x76;

#define DHTPIN 10
#define DHTTYPE DHT11
#define FAN_PIN 12

#define WSPEED 3
#define RAIN 2
#define WDIR A0

#define SAMPLING_TIME_MIN 1
#define SENDING_TIME_MIN 5

#define SF(x) String(F(x))

DHT dht(DHTPIN, DHTTYPE);

/*****************************************
 * Sensors definition
 ****************************************/
BH1750 lightMeter;
Adafruit_BME280 bme;
RTC_DS1307 rtc;
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature dstemp(&oneWire);

/*****************************************
 * Comunication and logging system
 ****************************************/
Drok com = Drok(Serial1, APN, APNUSER, PASS, "YWRtaW46QlYzWGp2clA=", "", false);
OpenLog sdLog = OpenLog(Serial2);
Istsos sos(sdLog, com, SERVER, URI, PROCEDURE_ID);

/******************************************
 * Global variable definition
 *****************************************/
uint16_t lux  = 0;
float pressure = 0.0;
float humidity = 0.0;
float temp = 0.0;
short soil = 0;
float intTemp = 0.0;

uint8_t lastMin = 0;
uint8_t lastDay = 0;
DateTime lastSendDate ;

//These are all the weather values that wunderground expects:
short winddir = 0; // [0-360 instantaneous wind direction]
float windspeedms = 0; // [m/s instantaneous wind speed]
volatile float lastrain = 0;
float rain = 0.0;

// char timezone[7];

// wind and rain variable
byte windClicks = 0;
volatile unsigned long lastWindCheck = 0;
volatile long lastWindIRQ = 0;
volatile unsigned long raintime, rainlast;
unsigned long lastRun, lastSend;
uint8_t lastMinute;


/******************************************
 * Function to monitor RAM usage
 *****************************************/
/*int freeRam() {
    extern int __heap_start, *__brkval;
    int v;
    return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}*/

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
        // init serial port
    Serial.begin(9600);
    Serial1.begin(57200);
    Serial2.begin(9600);
    delay(3000);

    if (!rtc.begin())
    {
        Serial.println(F("Couldn't find RTC"));
        return;
    }

    sos.begin();
    Serial.println(F("SOS ready"));

    Serial.println(F("Init..."));

    logMessage(SF("Start init process..."));

    // init sensors
    lightMeter.begin();
    Serial.println(F("light ready"));
    dht.begin();
    Serial.println(F("DHT ready"));

    if (!bme.begin(BME_I2C_ADDR))
    {
        BME_I2C_ADDR = 0x77;
        if(!bme.begin(BME_I2C_ADDR))
        {
            Serial.println(F("BME280 sensor not found"));
            logMessage(SF("BME280 sensor not found"));
            delay(30000);
            while (1) ;
        }

    }
    Serial.println(F("BME ready"));



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

    Serial.println(F("done"));

    logMessage(SF("init process success"));
    delay(5000);

    bool flag = false;

    while(!flag)
    {
        logMessage(SF("Start rtc sync process"));
        flag = syncRTC(com, rtc);
    }

    logMessage(SF("RTC sync success"));

    checkInternalTemp();

    logMessage(SF("start loop"));

    delay(5000);

    DateTime now = rtc.now();

    uint8_t min = now.minute();

    lastDay = now.day();
    lastMinute = min;

    lastRun = millis();
    lastSend = min; //millis();
    lastSendDate = now;
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

    lux = lightMeter.readLightLevel();
    // istsos max value = 9999
    if (lux >= 10000)
    {
        lux = 9999;
    }

    pressure = bme.readPressure() / 100.0F;
    humidity = bme.readHumidity();
    temp = dstemp.getTempCByIndex(0);
    soil = readSoil();

    winddir = get_wind_direction();
    windspeedms = get_wind_speed();

    intTemp = getInternalTemp();

    rain = lastrain;

    lastrain = 0;

}

/**
 * This function generate a String with the date and the measures
 *
 * @return String string containing date and measures
 */
String formatWeatherMeasure(const DateTime& now) {

    String message = getFormattedDate(now);

    message += "," + String(intTemp);
    message += "," + String(soil);
    message += "," + String(lux);
    message += "," + String(pressure);
    message += "," + String(humidity);
    message += "," + String(temp);
    message += "," + String(rain);
    message += "," + String(winddir);
    message += "," + String(windspeedms);

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

    bool res = sos.sendDataTest();

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

    // if((unsigned long)(millis() - lastRun) > SAMPLING_TIME)
    if(calcInterval(min, lastMinute, SAMPLING_TIME_MIN) && min < 60)
    {

        // lastRun = millis();
        lastMinute = min;

        getWeatherMeasure();

        String message = formatWeatherMeasure(now);
        Serial.println(message);

        sos.logData(message);

        checkInternalTemp();

        if(calcSendTime(now, lastSendDate, SENDING_TIME_MIN) || !sendStatus)
        {
            Serial.println(F("Time to send data"));
            sendData();

            if (count == 0)
            {
                lastSendDate = now;
            }
            delay(5000);
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
                now = rtc.now();

                lastDay = now.day();

                Serial.println(F("RTC sync ok"));
            }
        }
    }
    // back to sleep :)
    // LowPower.powerDown(SLEEP_8S, ADC_ON, BOD_ON);
    // LowPower.idle(SLEEP_8S, ADC_OFF, TIMER5_OFF, TIMER4_OFF, TIMER3_OFF, TIMER2_OFF, TIMER1_OFF, TIMER0_OFF, SPI_ON, USART3_OFF, USART2_OFF, USART1_OFF, USART0_OFF, TWI_OFF);
    delay(8000);

}
