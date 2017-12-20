BOARD_TAG   	= mega
BOARD_SUB	  	= atmega2560
USER_LIB_PATH 	= $(realpath ../libraries)
ARDUINO_DIR   	= /opt/arduino-1.8.4
ARDUINO_LIBS  	= Wire SPI OneWire \
				  Adafruit_BME280 Adafruit_Sensor \
				  istsos BH1750 \
				  DallasTemperature DHT RTClib RunningMedian

SERVER 			= '"geoservice.ist.supsi.ch"'
URI 			= '"/4onse/wa/istsos/services/sos/operations/fastinsert"'
PROCEDURE_ID 	= '"1ee64d2ae56411e7a93608002745029a"'
BASIC_AUTH		= '"YWRtaW46QlYzWGp2clA="'

APN				= '"gprs.swisscom.ch"'
APNUSER			= '"gprs"'
PASS			= '"gprs"'
SIM_PIN			= '""'


include ../Arduino.mk
