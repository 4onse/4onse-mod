


void sendCommand(String cmd)
{
    Serial.print("Send command: ");
    Serial.print(cmd);

    Serial1.print(cmd);
    Serial1.flush();

    Serial.println("-----SIM800-response-----");
    unsigned long start = millis();
    do
    {
        while(Serial1.available() > 0)
        {
            char c = Serial1.read();
            Serial.print(c);
        }
    } while( (unsigned long)(millis() - start) < 5000UL);
    Serial.println("-------------------------");
}

void setup()
{
    Serial.begin(9600);
    while(!Serial){}
    Serial1.begin(57200);
    while(!Serial1){}

    delay(1000);

    Serial.println(F("setup ready..."));
}

void loop()
{

    Serial.println("Start comunication");


    // sendCommand("AT+GCAP\r\n");
    // sendCommand("AT+GMI\r\n");
    // sendCommand("AT+GMM\r\n");
    sendCommand("AT+GMR\r\n");
    sendCommand("AT+CMEE=1\r\n");

    // init
    sendCommand("AT+CFUN=0\r\n");
    sendCommand("AT+CFUN=1,1\r\n");
    sendCommand("AT&FZE0\r\n");

    // get reg status
    sendCommand("AT+CREG?\r\n");

    // connect to GPRS
    sendCommand("AT+CIPSHUT\r\n");
    sendCommand("AT+SAPBR=3,1,\"Contype\",\"GPRS\"\r\n");
    sendCommand("AT+SAPBR=3,1,\"APN\",\"gprs.swisscom.ch\"\r\n");
    sendCommand("AT+SAPBR=3,1,\"USER\",\"gprs\"\r\n");
    sendCommand("AT+SAPBR=3,1,\"PWD\",\"gprs\"\r\n");
    sendCommand("AT+CGDCONT=1,\"IP\",\"gprs.swisscom.ch\"\r\n");
    sendCommand("AT+CGACT=1,1\r\n");
    sendCommand("AT+SAPBR=1,1\r\n");
    sendCommand("AT+SAPBR=2,1\r\n");
    sendCommand("AT+CGATT=1\r\n");
    sendCommand("AT+CIPMUX=1\r\n");
    sendCommand("AT+CIPQSEND=1\r\n");
    sendCommand("AT+CIPRXGET=1\r\n");
    sendCommand("AT+CSTT=\"gprs.swisscom.ch\",\"gprs\",\"gprs\"\r\n");
    sendCommand("AT+CIICR\r\n");
    sendCommand("AT+CIFSR;E0\r\n");
    sendCommand("AT+CDNSCFG=\"208.67.222.222\",\"208.67.220.220\"\r\n");

    // execute POST request
    sendCommand("AT+HTTPINIT\r\n");
    sendCommand("AT+HTTPPARA=\"URL\",\"https://geoservice.ist.supsi.ch\"\r\n");
    sendCommand("AT+HTTPPARA=\"CID\",1\r\n");
    sendCommand("AT+HTTPPARA=\"REDIR\",1\r\n");
    sendCommand("AT+HTTPPARA=\"CONTENT\",\"text/plain;charset=utf-8\"\r\n");
    sendCommand("AT+HTTPSSL=1\r\n");
    sendCommand("AT+HTTPSSL=?\r\n");
    sendCommand("AT+HTTPSSL?\r\n");

    while(1);

}
