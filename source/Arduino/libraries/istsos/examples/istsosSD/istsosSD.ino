#include <istsos.h>
#include <log/sdOpenlog.h>

OpenLog sd = OpenLog(Serial2);

void sleep()
{
  Serial.println(F("end..."));
  while (1)
  {
    delay(10000);
  }
}

void setup() {
  Serial.begin(9600);
  Serial2.begin(9600);
  delay(5000);

  Serial.print(F("Initialize..."));

  sd.begin();
  delay(2000);
  Serial.println(F("done"));

  delay(5000);
}

void loop() {

  sd.createDirectory("TEST");
  sd.cd("TEST");

  sd.createFile("TEST.TXT");
  sd.writeLine("TEST.TXT", "new line...");
  sd.createFile("TEST2.TXT");
  delay(200);

  String tmp = sd.ls();

  Serial.print(F("File: "));
  Serial.println(tmp);

  String content = sd.readFile("TEST.TXT");

  Serial.print(F("File content: "));
  Serial.print(content);

  sleep();

}
