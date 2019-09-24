#include <istsos.h>
#include <com/istsosWifi.h>

// arduino mega HardwareSerial
IstsosWifi istsos = IstsosWifi(Serial1, "1", "network_ssid", "network_Password");

const char server[] = "istsos.org";
const char uri[] = "/istsos/wa/istsos/services/demo/operations/fastinsert";


void setup() {
  // Set console baud rate
  Serial.begin(9600);
  Serial1.begin(19200);
  delay(5000);

  // init
  if(!istsos.begin())
    Serial.println(F("restart not wotking"));
  else
  {
    Serial.println(F("done"));
    delay(5000);
  }

}

void loop() {

  String message = "message";

  int response = istsos.executePost(server, uri, message);

  if (response != REQUEST_SUCCESS)
  {
    Serial.println(F("Request failure"));
    Serial.println(response);
  }
  else
  {
    Serial.println(F("Request success"));
  }


  while(1);

}
