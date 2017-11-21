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

### Build with makefile

#### Configure the makefile

Open the *Makefile* and set the ARDUINO_DIR

```
BOARD_TAG   	= mega
BOARD_SUB	  	= atmega2560
USER_LIB_PATH 	= $(realpath ../libraries)
ARDUINO_DIR   	= /opt/arduino-1.8.4
ARDUINO_LIBS  	= Wire SPI OneWire \
				  Adafruit_BME280 Adafruit_Sensor \
				  istsos LiquidCrystal  BH1750 \
				  DallasTemperature DHT LowPower RTClib

SERVER 			= '"geoservice.ist.supsi.ch"'
URI 			= '"/4onse/wa/istsos/services/sos/operations/fastinsert"'
PROCEDURE_ID 	= '"c264e6ba50cb11e79e2008002745029a"'
BASIC_AUTH		= '"basic_auth_token"'

APN				= '"gprs.swisscom.ch"'
APNUSER			= '"gprs"'
PASS			= '"gprs"'
SIM_PIN			= '"1234"'


include ../Arduino.mk
```

#### Build the code

```
make
```

#### Upload the software

```
make upload
```

### Build with arduino IDE  (WIP)

Download and install the arduino IDE

Open the arduino IDE -> Files -> Preferences, and set the Sketchbook location to the project root (Es):

```
/home/ist/workspace/4onse/4onse-ws/source/Arduino
```

Load the sketch *stationFull.ino* inside the folder stationFull, uncomment and set the following variable with the right value (around line 34):
```
#define SERVER "geoservice.ist.supsi.ch"
#define URI "/4onse/wa/istsos/services/sos/operations/fastinsert"
#define PROCEDURE_ID "175378da9a0511e79e2008002745029a"
#define BASIC_AUTH "asdsadsadsad"

#define APN "gprs.swisscom.ch"
#define APNUSER "gprs"
#define PASS "gprs"
#define SIM_PIN "1234"
```
Compile and upload the code
