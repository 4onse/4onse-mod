BOARD_TAG   	= mega
BOARD_SUB	  	= atmega2560
USER_LIB_PATH 	= $(realpath ../libraries)
ARDUINO_DIR   	= /opt/arduino-1.8.4
ARDUINO_LIBS  	= Wire SPI OneWire \
				  Adafruit_BME280 Adafruit_Sensor \
				  istsos BH1750 \
				  DallasTemperature DHT RTClib

SERVER 			= '"geoservice.ist.supsi.ch"'
URI 			= '"/4onse/wa/istsos/services/sos/operations/fastinsert"'
PROCEDURE_ID 	= '"175378da9a0511e79e2008002745029a"'

APN				= '"gprs.swisscom.ch"'
APNUSER			= '"gprs"'
PASS			= '"gprs"'


include ../Arduino.mk
