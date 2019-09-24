#include <istsos.h>
#include <com/sim800.h>

// arduino mega HardwareSerial
Sim800 istsos = Sim800(Serial1, "gprs.swisscom.ch", "gprs", "gprs");

const char server[] = "istsos.org";
const char uri[] = "/istsos/wa/istsos/services/demo/operations/fastinsert";


void setup() {
  // Set console baud rate
  Serial.begin(9600);
  Serial1.begin(57200);
  delay(5000);
  Serial.print(F("Init..."));

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

  // message = "procedure_id;YYYY-MM-DDTHH:MM:SS+ZZZZ,measure_0,measure_1,measure_2"
  String message = "fdcc53b23ec611e787870800273cbaca;2017-05-26T10:37:00+0200,24.57,74,979.24";

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

  //delay(10000);
  while(1);

}
