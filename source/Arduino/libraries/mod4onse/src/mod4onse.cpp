#include "mod4onse.h"

Mod4onse::Mod4onse()
{
    if(USE_DS18B20)
    {
        OneWire oneWire(ONE_WIRE_BUS);
        dstemp = DallasTemperature(&oneWire);
    }

    if(USE_BME280)
    {
        bme = Adafruit_BME280();
    }

    if(USE_BH1750)
    {
        lightMeter = BH1750();
    }
}

bool Mod4onse::begin()
{

    if(USE_BME280)
    {
        bool status = bme.begin(BME_I2C_ADDR);
        if (!status) {
            Serial.println(F("Could not find a valid BME280 sensor, check wiring!"));
            return false;
        }
    }

    if(USE_BH1750)
    {
        lightMeter.begin();
    }

    if (USE_SOIL)
    {
        pinMode(SOIL_A_PIN, INPUT);
    }

    return true;

}

short Mod4onse::get_wind_direction()
{
    unsigned short adc;
    adc = analogRead(WDIR); // get the current reading from the sensor

    // The following table is ADC readings for the wind direction sensor output, sorted from low to high.
    // Each threshold is the midpoint between adjacent headings. The output is degrees for that ADC reading.
    // Note that these are not in compass degree order! See Weather Meters datasheet for more information.

    if (adc < 380) return (113);
    if (adc < 393) return (68);
    if (adc < 414) return (90);
    if (adc < 456) return (158);
    if (adc < 508) return (135);
    if (adc < 551) return (203);
    if (adc < 615) return (180);
    if (adc < 680) return (23);
    if (adc < 746) return (45);
    if (adc < 801) return (248);
    if (adc < 833) return (225);
    if (adc < 878) return (338);
    if (adc < 913) return (0);
    if (adc < 940) return (293);
    if (adc < 967) return (315);
    if (adc < 990) return (270);
    return (-1); // error, disconnected?
}

String Mod4onse::getMeasures()
{
    String tmp = String("");

    if(USE_DS18B20)
    {
        dstemp.requestTemperatures();
        // flaot temp = dstemp.getTempCByIndex(0);
        tmp += "," + String(dstemp.getTempCByIndex(0));

    }

    if(USE_BME280)
    {
        float pressure = bme.readPressure() / 100.0F;
        float humidity = bme.readHumidity();
        tmp += "," + String(pressure);
        tmp += "," + String(humidity);

    }

    if(USE_BH1750)
    {
        lightMeter.configure(BH1750_ONE_TIME_HIGH_RES_MODE);
        delay(150);
        uint16_t lux = lightMeter.readLightLevel();
        // istsos max value = 9999
        if (lux >= 10000)
            lux = 9999;

        tmp += "," + String(lux);

    }

    if (USE_SOIL)
    {
        short value = analogRead(SOIL_A_PIN);
        short measure = ((1023.0 - value) / 1023.0) * 100.0;
        tmp += "," + String(measure);
    }

    if (USE_WDIR)
    {
        tmp += "," + String(this->get_wind_direction());
    }

    return tmp;
}
