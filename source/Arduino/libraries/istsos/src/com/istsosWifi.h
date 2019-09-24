#ifndef ISTSOSWIFI_H
#define ISTSOSWIFI_H
#define TIMEOUT 1500
#define REQUEST_SUCCESS 0
#define REQUEST_FAILURE 1
#define CONNECTION_PROBLEM 2
#define NETWORK_FAILURE 3
#define WIFI_FAILURE 4
#define SERVER_FAILURE 5
#define RESET_FAILURE 6
#define CHECK_FAILURE 7
#define CWMODE_FAILURE 8
#define NTP_GMT_FAILURE 9
#define NTP_GMT_SUCCESS 10

#include <Arduino.h>
#include "comunication.h"

class IstsosWifi: public ICom
{
private:

  // generic for HardwareSerial or SoftwareSerial
  Stream* port;
  const char* cwmod;
  const char* ssid;
  const char* password;
  // variables to check status
  bool check;
  int success = 0;

  /**
      This is a function used to send command to the esp8266.

      @cmd AT command
      @strTrue text to check the success of the AT command
      @errorText text to check the failure of the AT command
      @errCode error code (see istsoswifi.h defined variables)

  */
  void sendCmd(String cmd, String strTrue, String errorText, uint8_t errCode);

  /**

      This function is used to check if the esp8266 response is equal to the string passed

      @strTrue success string to check
  */
  void checkResponse(String strTrue);

public:

  /**
      Constructor with Stream.

      @param portEsp Hardware or Software serial
      @param cwmod esp8266 mode (1 - client, 2 - AP, 3 - both)
      @param ssid name of the WLAN
      @param password password of the WLAN
  */
  IstsosWifi(Stream &portEsp, const char* cwmod, const char* ssid, const char* password);
  /**
      Initialize ESP8266 enviroment.
      Call it inside setup()

      @return bool TODO
  */
  uint8_t begin();

  /**
      Execute POST requests.

      @param server name of the istSOS server (e.g. istsos.org)
      @param uri uri for sending the request (e.g. /istsos/wa/istsos/services/demo/operations/fastinsert)
      @param data string of data formatted according to the istSOS fastInsert specs
  */
  uint8_t executePost(const char server[], const char uri[], const String& data);
  /**
      Update the internal RTC and return the current date.

      @param ntpServer[] name of the ntp server
      @param GMT number of the GMT

      @return uint32_t* epoch
  */
  uint32_t* ntpUpdate(const char ntpServer[], int GMT);
};

#endif
