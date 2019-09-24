#include "istsosWifi.h"

IstsosWifi::IstsosWifi(Stream &portEsp, const char* cwmod, const char* ssid, const char* password)
{
    this->port = &portEsp;

    this->cwmod = cwmod;
    this->ssid = ssid;
    this->password = password;
}

void IstsosWifi::sendCmd(String cmd, String strTrue, String errorText, uint8_t errCode)
{
  delay(TIMEOUT);
  int retries = 0;
  while(check == false || retries <= 5) {
    delay(TIMEOUT);
    this->port->println(cmd);
    this->checkResponse(strTrue);
    if (check) {
      this->success = REQUEST_SUCCESS;
      break;
    } else if (retries == 5) {
      this->success = errCode;
      retries++;
      break;
    } else {
      retries++;
    }
  }
  retries = 0;
}

uint8_t IstsosWifi::begin()
{
  this->sendCmd("AT","OK", "ERROR", CHECK_FAILURE);
  if (this->success == REQUEST_SUCCESS) {
    this->sendCmd("AT+RST","ready","ERROR", RESET_FAILURE);
  }
  if (this->success == REQUEST_SUCCESS) {
    this->sendCmd("AT+CWMODE=" + String(this->cwmod), "OK","ERROR", CWMODE_FAILURE);
  }
  if (this->success == REQUEST_SUCCESS) {
    this->sendCmd("AT+CWJAP=\"" + String(this->ssid) +"\",\"" + String(this->password) + "\"","WIFI GOT IP","ERROR", WIFI_FAILURE);
  }
  return this->success;
}

void IstsosWifi::checkResponse(String strTrue)
{
  String espOutput = "";
  String endString = "";
  this->check = false;
  //bool progress = false;
  unsigned long lastRead = millis();   // last time a char was available
  while (millis() - lastRead < TIMEOUT){
    while(this->port->available() && this->check==false) {
      char character = this->port->read();
      if (character == '\r') {
        endString.concat(character);
      } else if ( character == '\n') {
        espOutput = "";
        endString.concat(character);
        /*if (endString == "\r\n") {
          progress = true;
        }*/
      } else {
        espOutput.concat(character);
      }
      if (espOutput == strTrue) {
        this->check = true;
      } else if (espOutput == "ERROR") {
        this->check = false;
      } else {
        this->check = false;
      }
    }
  }
}

uint8_t IstsosWifi::executePost(const char server[], const char uri[], const String& data)
{
  this->success = 0;
  String espOutput = "";
  this->sendCmd("AT+CIPSTART=\"TCP\",\"" + String(server) + "\",80","OK","ERROR",CONNECTION_PROBLEM);
  if (this->success == REQUEST_SUCCESS) {
    int headerLen = 162 + data.length();
    //String cipSend = "";
    this->port->print(F("AT+CIPSEND="));
    this->port->println(headerLen);
    this->checkResponse(">");
    if (this->check) {
      Serial.println(F("Sending.."));
      this->port->print("POST " + String(uri) + " HTTP/1.1\r\n");
      this->port->print("Host: " + String(server) + "\r\n");
      this->port->print(F("Accept: */*\r\n"));
      this->port->print(F("Content-Length: "));
      this->port->print(data.length());
      this->port->print(F("\r\n"));
      this->port->print(F("Content-Type: text/plain;charset:utf-8\r\n\r\n"));
      this->port->print(data);
      espOutput = "";
      boolean startMarker = false;
      boolean endWhile = false;
      int brackets = 0;
      unsigned long lastRead = millis();
      while (millis() - lastRead < TIMEOUT && endWhile == false){
        while(this->port->available()) {
          char character = this->port->read();
          if (character == '{') {
            startMarker = true;
          }
          if (startMarker) {
            if (character == '{') {
              brackets++;
            } else if (character=='}') {
              brackets--;
            }
            espOutput.concat(character);
            if (brackets==0) {
              endWhile = true;
            }
          }
        }
      }
      if (espOutput.indexOf("success\": true") >=0) {
        this->sendCmd("AT+CIPCLOSE","CLOSED","ERROR",CONNECTION_PROBLEM);
        this->success = REQUEST_SUCCESS;
      } else {
        this->sendCmd("AT+CIPCLOSE","CLOSED","ERROR",CONNECTION_PROBLEM);
        this->success = REQUEST_FAILURE;
      }
    }
    return this->success;
  } else {
    return  this->success;
  }
}

uint32_t* IstsosWifi::ntpUpdate(const char ntpServer[], int GMT)
{
  const byte NTP_PACKET_SIZE = 48; // NTP time stamp is in the first 48 bytes of the message
  byte packetBuffer[ NTP_PACKET_SIZE];
  this->sendCmd("AT+CIPSTART=\"UDP\",\"" + String(ntpServer) + "\",123","OK","ERROR",CONNECTION_PROBLEM);
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011; // LI, Version, Mode
  packetBuffer[1] = 0; // Stratum, or type of clock
  packetBuffer[2] = 6; // Polling Interval
  packetBuffer[3] = 0xEC; // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12] = 49;
  packetBuffer[13] = 0x4E;
  packetBuffer[14] = 49;
  packetBuffer[15] = 52;
  this->port->print("AT+CIPSEND=");
  this->port->println(NTP_PACKET_SIZE);
  this->checkResponse(">");
  if (this->check) {
    for (uint32_t i = 0; i < NTP_PACKET_SIZE; i++) {
      this->port->write(packetBuffer[i]);
    }
  }
  int counta = 0;
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  this->checkResponse("+IPD,48:");
  if (this->check) {
    int i = 0;
    while (this->port->available() > 0) {
      byte ch = this->port->read();
      if (i < NTP_PACKET_SIZE       )
      {
        packetBuffer[i] = ch;
      }
      i++;
      if ( ( i < NTP_PACKET_SIZE ) && ( this->port->available() == 0 ) )
      {
        while (this->port->available() == 0) // you may have to wait for some bytes
        {
          counta += 1;
          delay(100);
          if (counta == 15) {
            exit;
          }
        }
      }
    }
  }
   //the timestamp starts at byte 40 of the received packet and is four bytes,
  // or two words, long. First, esxtract the two words:

  uint32_t highWord = word(packetBuffer[40], packetBuffer[41]);
  uint32_t lowWord = word(packetBuffer[42], packetBuffer[43]);
  // combine the four bytes (two words) into a long integer
  // this is NTP time (seconds since Jan 1 1900):
  uint32_t secsSince1900 = highWord << 16 | lowWord;
  //Serial.print("Seconds since Jan 1 1900 = " );
  //Serial.println(secsSince1900);

  // now convert NTP time into everyday time:
  // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
  const uint32_t seventyYears = 2208988800UL;
  // subtract seventy years:
  uint32_t epoch = secsSince1900 - seventyYears + (GMT*3600);
  this->sendCmd("AT+CIPCLOSE","CLOSED","ERROR",CONNECTION_PROBLEM);
  uint32_t* result = new uint32_t[1];
  result[0] = epoch;
  return result;
}
