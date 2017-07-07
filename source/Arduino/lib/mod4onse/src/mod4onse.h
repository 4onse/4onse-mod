#ifndef MOD4ONSE_H
#define MOD4ONSE

#include "Arduino.h"

#include <OneWire.h>
#include <DallasTemperature.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#include <BH1750.h>


#define USE_DS18B20 true
#define USE_BME280 true
#define USE_BH1750 true
#define USE_SOIL true
#define USE_WDIR true

#define ONE_WIRE_BUS 10
#define BME_I2C_ADDR 0x76
#define SOIL_A_PIN A13
#define WDIR A0

class Mod4onse
{

private:

    DallasTemperature dstemp;
    Adafruit_BME280 bme;
    BH1750 lightMeter;
    short get_wind_direction();
    float get_wind_speed();

public:

    Mod4onse();

    bool begin();

    String getMeasures();


};


#endif
