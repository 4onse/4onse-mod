
#include "istsos.h"

Istsos::Istsos(ILog &sd, ICom &com, const char* server, const char* resource, const char* procedure)
{
    this->sd = &sd;
    this->com = &com;

    this->server = server;
    this->resource = resource;
    this->procedure = procedure;

}

uint8_t Istsos::beginSD()
{
    // init sd
    bool flag = this->sd->begin();

    if (!flag)
    {
        return false;
    }
    return true;
}

uint8_t Istsos::beginSIM800()
{
    // init comm
    bool flag = this->com->begin();

    if(!flag)
    {
        return false;
    }

    this->com->useAutoDisconnect(true);
    this->com->useSSL(false);

    return true;
}

uint8_t Istsos::sendData()
{
    uint8_t returnCode = 0;

    // uint8_t status = 0; // this->com->connect();
    //
    // if( status != 0)
    // {
    //     return status;
    // }

    this->sd->cd("TMP");

    unsigned long tmp = millis();

    // while(1)
    while((unsigned long)(millis() - tmp) < 300000UL)
    {
        String fileName = this->sd->getFirstFileName();

        if (fileName == String(""))
        {
            break;
        }
        else if (fileName.indexOf("L6") >= 0 || fileName.indexOf("SLOG") >= 0)
        {
            this->sd->removeFile(fileName);
            #ifdef DEBUG_ISTSOS
                Serial.println(F("removed file with wrong date"));
            #endif
            continue;
        }

        returnCode = this->sendSingleFile(fileName);

        if(returnCode == 0)
        {
            #ifdef DEBUG_ISTSOS
                Serial.println(F("Transmission ok, delete file"));
            #endif

            this->sd->copyToLog(fileName);
            this->sd->removeFile(fileName);
        }else{
            // this->com->disconnect();
            break;
        }
    }

    this->sd->cd("..");

    #ifdef DEBUG_ISTSOS
        Serial.println(F("Transmission ended...."));
    #endif

    // this->com->disconnect();

    return returnCode;
}

uint8_t Istsos::sendSingleFile(const String& fileName)
{
    String message = this->procedure;

    message += ";" + this->sd->readFile(fileName);

    // format for istsos requests;
    message.replace("\r\n", "");
    message.replace("\n\r", "@");
    message.remove(message.length() - 1);


    #ifdef DEBUG_ISTSOS
        Serial.print(F("Procedure id: "));
        Serial.println(this->procedure);
        Serial.print(F("Read from : "));
        Serial.println(fileName);
        Serial.println(F("Message to send: "));
        Serial.println(message);
    #endif

    uint8_t code = this->com->executePost(this->server, this->resource, message);
    if (code == 0)
    {
        return code;
    }

    #ifdef DEBUG_ISTSOS
        Serial.print(F("Error code: "));
        Serial.println(code);
    #endif

    return code;
}

void Istsos::logData(const String& message)
{

    String tmp = getValue(message, ',', 0);
    String date = getValue(tmp, 'T', 0);
    String time = getValue(tmp, 'T', 1);

    tmp.remove(0);

    String fileName = "L";
    fileName += getValue(date, '-', 0).substring(2);

    if (fileName == String(F("L00")))
    {
        return;
    }

    fileName += String(getValue(date, '-', 1).toInt(), HEX);
    fileName += getValue(date, '-', 2);
    fileName += getValue(time, ':', 0);
    fileName += ".TXT";

    #ifdef DEBUG_ISTSOS
        Serial.print(F("Filename: "));
        Serial.println(fileName);
    #endif

    this->sd->cd("TMP");
    this->sd->writeLine(fileName, message);
    this->sd->cd("..");
}

void Istsos::logging(const String& date, const String& message)
{
    // this->sd->cd("..");
    String tmpDate = getValue(date, 'T', 0);

    String fileName = "SLOG";
    fileName += getValue(tmpDate, '-', 0).substring(2);
    fileName += String(getValue(tmpDate, '-', 1).toInt(), HEX);
    fileName += ".TXT";

    String logMessage = date + ": " + String(message);

    this->sd->cd("SLOG");
    this->sd->writeLine(fileName, logMessage);
    this->sd->cd("..");
}

uint32_t* Istsos::ntpUpdate(const char ntpServer[], int GMT)
{
    return this->com->ntpUpdate(ntpServer, GMT);
}

bool Istsos::checkMissingData()
{
    this->sd->cd("TMP");
    String tmp = this->sd->getFirstFileName();
    this->sd->cd("..");

    if(tmp != String(""))
    {
        return true;
    }
    return false;

}
