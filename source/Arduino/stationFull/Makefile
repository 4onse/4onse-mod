BOARD_TAG   	= mega
BOARD_SUB	  	= atmega2560
USER_LIB_PATH 	= $(realpath ../lib)
ARDUINO_DIR   	= /opt/arduino-1.8.4
ARDUINO_LIBS  	= Wire SPI OneWire \
				  Adafruit_BME280 Adafruit_Sensor \
				  istsos BH1750 \
				  DallasTemperature DHT LowPower RTClib

SERVER 			= '"geoservice.ist.supsi.ch"'
URI 			= '"/4onse/wa/istsos/services/sos/operations/fastinsert"'
PROCEDURE_ID 	= '"c264e6ba50cb11e79e2008002745029a"'

APN				= '"gprs.swisscom.ch"'
APNUSER			= '"gprs"'
PASS			= '"gprs"'


include ../Arduino.mk
