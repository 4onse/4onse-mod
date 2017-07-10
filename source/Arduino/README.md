# arduino station code

## REQUIREMENTS

### Hardware

this code is compatible only with the Arduino MEGA 2560.

### Software

* [RTCLib](https://github.com/adafruit/RTClib)
* [Adafruit_BME280_Library](https://github.com/adafruit/Adafruit_BME280_Library)
* [Adafruit_Sensor](https://github.com/adafruit/Adafruit_Sensor)
* [BH1750](https://github.com/mysensors/MySensorsArduinoExamples/tree/master/libraries/BH1750)
* [DallasTemperature](https://github.com/milesburton/Arduino-Temperature-Control-Library)
* [DHT](https://github.com/adafruit/DHT-sensor-library)
* [istsos](https://gitlab.com/ist-supsi/arduino-istsos)

## GETTING STARTED

TODO

### Configure the makefile

Open the *Makefile* and set the ARDUINO_DIR

```
BOARD_TAG   	= mega
BOARD_SUB	  	= atmega2560
USER_LIB_PATH 	= $(realpath ../lib)
ARDUINO_DIR   	= /opt/arduino-1.8.3
ARDUINO_LIBS  	= Wire SPI OneWire \
				  Adafruit_BME280 Adafruit_Sensor \
				  istsos LiquidCrystal  BH1750 \
				  DallasTemperature DHT LowPower RTClib

include ../Arduino.mk
```

### Build the code

```
make
```


### Upload the software

```
make upload
```
