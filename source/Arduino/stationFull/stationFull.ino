
// istsos comunication library (GPRS)
#include <istsos.h>
#include <com/sim800.h>
//#include <log/sdIstsos.h>
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
// #include <LiquidCrystal.h>
#include "functions.h"

/*****************************************
 * Define sensor pin
 ****************************************/
#define ONE_WIRE_BUS 9
#define SOIL_A_PIN A13
#define BME_I2C_ADDR 0x76

#define DHTPIN 10
#define DHTTYPE DHT11
#define FAN_PIN 12

#define WSPEED 3
#define RAIN 2
#define WDIR A0
// const byte WSPEED = 3;
// const byte RAIN = 2;
// const byte WDIR = A0;

// time between different measures
#define SAMPLING_TIME 1
// time to send the data
#define SENDING_TIME 10

#define SF(x) String(F(x))

// LiquidCrystal lcd(41, 39, 37, 35, 33, 31);
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
Sim800 com = Sim800(Serial1, APN, APNUSER, PASS, "YWRtaW46QlYzWGp2clA=");
//SdIstsos sdLog = SdIstsos(Serial2);
OpenLog sdLog = OpenLog(Serial2);

//const char server[] = {"geoservice.ist.supsi.ch"};
//const char uri[] = {"/4onse/wa/istsos/services/sos/operations/fastinsert"};
// const char procedure[] = {"c264e6ba50cb11e79e2008002745029a"};  // {"64341ccc479911e79e2008002745029a"};

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

//These are all the weather values that wunderground expects:
short winddir = 0; // [0-360 instantaneous wind direction]
float windspeedms = 0; // [m/s instantaneous wind speed]
float lastrain = 0;
float rain = 0.0;

char timezone[7];

// wind and rain
// unsigned long lastRun = 0UL;
uint8_t lastSend = 0;

byte windClicks = 0;
volatile unsigned long lastWindCheck = 0;
volatile long lastWindIRQ = 0;
volatile unsigned long raintime, rainlast;


/******************************************
 * Function to monitor RAM usage
 *****************************************/
/*int freeRam () {
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

 bool logTik = false;
void rainIRQ()
{
    raintime = millis(); // grab current time

    if ((unsigned long)(millis() - rainlast) > 10) // ignore switch-bounce glitches less than 10mS after initial edge
    {
        lastrain += 0.2; //0.011;

        rainlast = millis(); // raintime; // set up for next event
        //TODO log tik date
        logTik= true;
        //sos.tikLogging(getFormattedDate(rtc.now(), timezone));
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

    // https://www.sparkfun.com/datasheets/Sensors/Weather/Weather%20Sensor%20Assembly..pdf
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

template <typename T>
void printToLcd(const T message, uint8_t col=0, uint8_t row=0,  bool clear=true)
{
    return;
    //if (clear)
    //lcd.clear();

    //lcd.setCursor(col,row);
    //lcd.print(message);
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
    float t = getInternalTemp();

    printToLcd(String("int temp: ") + String(t), 0, 1, false);

    // remove return to start the fan
    return;

    if (t >= 40.0)
    {
        Serial.println(F("Start fan...."));
        digitalWrite(FAN_PIN, LOW);
        delay(20000);
        Serial.println(F("Stop fan..."));
        digitalWrite(FAN_PIN, HIGH);
    }
}


void logMessage(const String message)
{
    String date = getFormattedDate(rtc.now(),timezone);

    sos.logging(date, message);
}

void setup() {
        // init serial port
    Serial.begin(9600);
    Serial1.begin(57200);
    Serial2.begin(9600);
    delay(3000);

    if (!rtc.begin()) {
        Serial.println(F("Couldn't find RTC"));
        return;
    }

    Serial.println(SERVER);
    Serial.println(APN);
    // Serial.println(APNUSER);
    // Serial.println(PASS);
    sos.begin();
    Serial.println(F("SOS ready"));

    Serial.println(F("Init..."));

    logMessage(SF("Start init process...."));

    //lcd.begin(16, 2);   // Number of columns, Number of rows of the LCD

    // init sensors
    lightMeter.begin();
    Serial.println(F("light ready"));
    dht.begin();
    Serial.println(F("DHT ready"));


    bool status = bme.begin(BME_I2C_ADDR);
    if (!status) {
        Serial.println(F("Could not find a valid BME280 sensor, check wiring!"));
        while (1) ;
    }
    Serial.println(F("BME ready"));



    // set pin mode
    pinMode(SOIL_A_PIN, INPUT);

    pinMode(WSPEED, INPUT_PULLUP);
    pinMode(RAIN, INPUT_PULLUP);
    pinMode(FAN_PIN, OUTPUT);
    digitalWrite(FAN_PIN, HIGH);

    // attach external interrupt pins to IRQ functions (rain and wind speed)
    attachInterrupt(digitalPinToInterrupt(RAIN), rainIRQ, FALLING);
    attachInterrupt(digitalPinToInterrupt(WSPEED), wspeedIRQ, FALLING);

    // turn on interrupts
    interrupts();

    Serial.println(F("done"));

    logMessage(SF("init process success"));
    delay(5000);

    logMessage(SF("Start rtc sync process"));
    // sync RTC
    bool flag = syncRTC(com, rtc, timezone);

    if(!flag)
    {
        Serial.print(F("Problem sync RTC..."));
        while(1) ;
    }

    logMessage(SF("RTC sync success"));

    lastMin = rtc.now().minute();
    lastSend = rtc.now().minute();
    lastDay = rtc.now().day();

    printToLcd(F("Ready to start..."));

    checkInternalTemp();

    logMessage(SF("start loop"));
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
            lux = 9999;
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
String formatWeatherMeasure() {

    String message = getFormattedDate(rtc.now(), timezone);

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
    String date = getFormattedDate(rtc.now(), timezone);
    if (count == 0)
        lastSend = rtc.now().minute(); // millis();

    printToLcd(F("Sending data..."));

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

    delay(1000);
    printToLcd(F("waiting..."));
}

void loop() {

    // check if it's time to log data
    uint8_t min = rtc.now().minute();


    if(logTik)
    {
        Serial.println(getFormattedDate(rtc.now(), timezone));
        logTik = false;
    }

    if(calcInterval(min, lastMin, SAMPLING_TIME))
    {
        //lastRun = millis();
        printToLcd(F("Reading data..."));

        getWeatherMeasure();

        String message = formatWeatherMeasure();
        Serial.println(message);

        sos.logData(message);

        printToLcd(F("waiting..."));

        lastMin = min;
        checkInternalTemp();

        if (calcInterval(min, lastSend, SENDING_TIME) || !sendStatus)
        {
            sendData();
        }

        // sync RTC every day
        if(rtc.now().day() != lastDay && sendStatus)
        {
            bool flag = syncRTC(com, rtc, timezone);

            if(!flag)
            {
                Serial.println(F("Problem sync RTC..."));
            }
            else
            {
                lastDay = rtc.now().day();
            }
        }
    }
    //LowPower.powerSave(SLEEP_8S, ADC_OFF, BOD_OFF, TIMER2_OFF);

}
