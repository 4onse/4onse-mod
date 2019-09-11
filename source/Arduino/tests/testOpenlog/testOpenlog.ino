
/*
 
 Connect the following OpenLog to Arduino:
 RXI of OpenLog to pin 2 on the Arduino
 TXO to 3
 GRN to 4
 VCC to 5V
 GND to GND
 
 */


//SoftwareSerial(rxPin, txPin)

int resetOpenLog = 36; //This pin resets OpenLog. Connect pin 4 to pin GRN on OpenLog.
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

int statLED = 13;

bool command;

void setup() {                
  pinMode(statLED, OUTPUT);
  Serial.begin(9600);

//  Serial.println("Find OpenLog Firmware Version");
//
  setupOpenLog(); //Resets logger and waits for the '<' I'm alive character
//  Serial.println("OpenLog online");
//
//  // gotoCommandMode(); //Puts OpenLog in command mode
//  Serial.println("here");
//  Serial2.println('?'); //Send a character to get help menu
//  waitToCommand();
//  Serial2.println("set 3");
//  waitToCommand();
  Serial2.println("cd TMP");
  waitToCommand();
  
  Serial2.println("ls");
  getValue(Serial2.readStringUntil('\n'), ' ', 0);
  String tmp = getValue(Serial2.readStringUntil('\n'), ' ', 0);
  String next_tmp = getValue(Serial2.readStringUntil('\n'), ' ', 0);
  while(next_tmp.length() > 1) {
      if (tmp > next_tmp) {
          tmp = next_tmp;
      }
      Serial.println(tmp);
      next_tmp = getValue(Serial2.readStringUntil('\n'), ' ', 0);
  }
  Serial.println(tmp);
//  String empty = getValue(Serial2.readStringUntil('\n'), ' ', 0);
//  String filename = getValue(Serial2.readStringUntil('\n'), ' ', 0);
//  Serial.println("READ: " + filename);
//  String tmp = getValue(Serial2.readStringUntil('\n'), ' ', 0);
//  Serial.println("READ: " + tmp);
//  //waitToCommand();
//  while(filename > tmp) {
//    filename = tmp;
//    tmp = getValue(Serial2.readStringUntil('\n'), ' ', 0);
//    Serial.println("Next tmp: " + tmp);
//  }
//  Serial.println("File giusto: " + filename);
}

void loop() {
  //Do nothing
}

//Setups up the software serial, resets OpenLog so we know what state it's in, and waits
//for OpenLog to come online and report '<' that it is ready to receive characters to record
void setupOpenLog(void) {
  pinMode(resetOpenLog, OUTPUT);
  Serial2.begin(9600);

  //Reset OpenLog
  digitalWrite(resetOpenLog, LOW);
  delay(100);
  digitalWrite(resetOpenLog, HIGH);

  //Wait for OpenLog to respond with '<' to indicate it is alive and recording to a file
  int no = 0;
  String bytes = "0";
  bool stat = 1;
  while(stat) {
    while(Serial2.available())
      {
        char c = Serial2.read();
        Serial.print(c);
        if (c == '\n')
          {
            no++;
          }
        else if (no >= 1 && isdigit(c))
          {
            bytes += c;
          }
        else if (c == '>')
          {
            stat = 0;
            break;
          }
        else if (c == '<')
          {
            stat = 0;
            break;
          }
      }
  }
  delay(1);
}

//This function pushes OpenLog into command mode
void gotoCommandMode(void) {
  //Send three control z to enter OpenLog command mode
  //Works with Arduino v1.0
  //Reset OpenLog
  Serial2.write(26);
  Serial2.write(26);
  Serial2.write(26);

  

  //Wait for OpenLog to respond with '<' to indicate it is alive and recording to a file
  int no = 0;
  String bytes = "0";
  bool stat = 1;
  while(stat) {
    while(Serial2.available())
      {
        char c = Serial2.read();
        Serial.print(c);
        if (c == '\n')
          {
            no++;
          }
        else if (no >= 1 && isdigit(c))
          {
            bytes += c;
          }
        else if (c == '>')
          {
            stat = 0;
            break;
          }
      }
  }
  delay(1);
}

bool waitToCommand()
{
    unsigned long timeout = millis();

    bool flag = false;
    while ((unsigned long)(millis() - timeout) < 5000UL)
    {

        if(Serial2.available())
        {
            char c = Serial2.read();
            Serial.print(c);
        }
    }
    return flag;
}

String getValue(const String& data, const char separator, const int index)
{
    int found = 0;
    int strIndex[] = { 0, -1 };
    int maxIndex = data.length() - 1;

    for (int i = 0; i <= maxIndex && found <= index; i++)
    {
        if (data.charAt(i) == separator || i == maxIndex)
        {
            found++;
            strIndex[0] = strIndex[1] + 1;
            strIndex[1] = (i == maxIndex) ? i+1 : i;
        }
    }
    return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}
