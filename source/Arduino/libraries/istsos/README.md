# istsos comunication lib 

Istsos comunication and logging library.

This library is designed to log data and cominucate with an istsos server.

Comunication with:

* GPRS (SIM800, SIM900)
* WiFi (EPS8266)

Logging data to an SD:

* [OpenLog](https://learn.sparkfun.com/tutorials/openlog-hookup-guide)


## REQUIREMENTS

To comunicate with the GPRS:

* [tinyGSM](https://github.com/vshymanskyy/TinyGSM)


## GETTING STARTED

### Comunication

Comunication using GPRS and Wifi.

#### GPRS

Comunication using SIM900 or SIM800.

Scatch example to comunicate using GPRS:

```
	examples/istsosGprs/istsosGprs.ino
```

#### WiFi

Comunication using ESP8266.

Scatch example to comunicate using Wifi:

```
	examples/istsosWifi/istsosWifi.ino
```

### Logging

#### OpenLog

Logging using OpenLog

Scatch example to use sd:

```
	examples/istsosSD/istsosSD.ino
```
