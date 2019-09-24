#include "sdIstsos.h"

SdIstsos::SdIstsos(Stream &serial)
{
    this->openLog = &serial;
};

bool SdIstsos::begin(){
    pinMode(RESET_OPENLOG, OUTPUT);
    //this->openLog->begin(9600);
    delay(200);

    this->reset();

    return this->init();
};

void SdIstsos::gotoCommandLine(){

    if (this->command)
        return;

    this->openLog->write(26);
    this->openLog->write(26);
    this->openLog->write(26);

    this->waitToCommand();

};

bool SdIstsos::reset()
{
    digitalWrite(RESET_OPENLOG, LOW);
    delay(500);
    digitalWrite(RESET_OPENLOG, HIGH);
    this->waitToCommand();
    return true;
};

bool SdIstsos::init()
{
    this->sendCommand("init");
    this->waitToCommand();

    return true;
}

bool SdIstsos::createFile(const String& fileName)
{
    // check if file already exists
    int size = this->getFileSize(fileName);
    if (size == -1)
        return true;

    this->sendCommand("new " + fileName);
    this->waitToCommand();
    delay(5);
    return true;
};



String SdIstsos::readFile(const String& fileName)
{

    if(DEBUG_SD)
    {
        Serial.print(F("Read file: "));
        Serial.println(fileName);
    }

    this->sendCommand("read " + fileName);

    while(1)
    {
        if(this->openLog->available())
            if(this->openLog->read() == '\n') break;
    }
    String message = "";

    for(int timeOut = 0 ; timeOut < 1000 ; timeOut++)
    {
        while(this->openLog->available())
        {
            while(this->openLog->available())
            {
                char c = this->openLog->read();
                if (c != '>')
                    message += c;
            }
            timeOut = 0;
        }
        delay(1);
    }

    return message;
};

void SdIstsos::sendCommand(const String& command)
{
    //this->clearBuffer();
    this->gotoCommandLine();
    this->openLog->println(command);
}

template<typename... Args>
void SdIstsos::sendCommandTest(Args... commands)
{
    this->gotoCommandLine();
    this->writeCommand(commands...);
    this->openLog->println();
}

template<typename T, typename... Args>
void SdIstsos::writeCommand(T com, Args... command)
{
    this->openLog->print(com);
    this->writeCommand(command...);
}

template<typename T>
void SdIstsos::writeCommand(T com)
{
    this->openLog->print(com);
}


void SdIstsos::clearBuffer()
{
    while(this->openLog->available() > 0) {
        this->openLog->read();
        delay(1);
    }
}

int SdIstsos::getFileSize(const String& fileName)
{
    this->sendCommand("size " + fileName);

    int no = 0;
    String bytes = "0";
    for(int timeOut = 0 ; timeOut < 1000 ; timeOut++)
    {
        while(this->openLog->available())
        {
            while(this->openLog->available())
            {
                char c = this->openLog->read();
                if (c == '\n'){
                    no++;
                }
                else if (no >= 1 && isdigit(c)){
                    bytes += c;
                }else if (c == '>'){
                    this->command = true;
                    break;
                }
            }
            timeOut = 0;
        }
        delay(1);
    }

    // if (DEBUG_SD)
    //     Serial.println("string: " + bytes + " end string");
    delay(5);
    return bytes.toInt();
}

void SdIstsos::writeLine(const String& file, const String& line)
{
    int size = this->getFileSize(file);

    if(DEBUG_SD)
    {
        Serial.print(F("Size: "));
        Serial.println(size);
    }

    if (size == 0)
    {
        this->createFile(file);
    }

    this->sendCommand("write " + file + " " + size);
    this->waitForEdit();

    this->openLog->println(line);
    this->openLog->println();

    //this->waitToCommand();

    //this->gotoCommandLine();
    this->command = true;
    delay(10);
}

void SdIstsos::removeFile(const String& file)
{
    //this->cd("TMP");
    this->sendCommand("rm " + file);
    this->waitToCommand();
    //this->cd("..");
}

String SdIstsos::ls()
{
    this->sendCommand("ls");

    String message = "";

    for(int timeOut = 0 ; timeOut < 1000 ; timeOut++)
    {
        while(this->openLog->available())
        {
            while(this->openLog->available())
            {
                String tmp = getValue(this->openLog->readStringUntil('\n'), ' ', 0);
                Serial.println(tmp);

                if (tmp[0] != '>' && tmp[0] != '\r')
                {
                    if(message == "")
                        message += tmp;
                    else
                        if (tmp.length() != 0 && (tmp.charAt(tmp.length() -1 ) != '/'))
                            message += "," + tmp;
                }
            }
            timeOut = 0;
        }
        delay(1);
    }
    return message;
}

String SdIstsos::getFirstFileName()
{
    this->sendCommand("ls");

    for(int timeOut = 0 ; timeOut < 1000 ; timeOut++)
    {
        while(this->openLog->available())
        {
            while(this->openLog->available())
            {
                String tmp = getValue(this->openLog->readStringUntil('\n'), ' ', 0);
                tmp.replace("\r", "");
                char lastChar = tmp.charAt(tmp.length() -1 );
                if (lastChar != '>' && tmp.length() != 0 && ( lastChar != '/'))
                {
                    this->waitToCommand();
                    return tmp;
                }
            }
            timeOut = 0;
        }
        delay(1);
    }
    //this->waitToCommand();
    this->clearBuffer(); //this->waitToCommand();
    return String("");
}

bool SdIstsos::openFile(const String& fileName)
{
    this->createFile(fileName);
    this->sendCommand("append " + fileName);
    this->waitForEdit();
    return true;
}

bool SdIstsos::createDirectory(const String& dirName)
{
    this->sendCommand("md " + dirName);
    this->waitToCommand();
    return true;
}

bool SdIstsos::copyToLog(const String& fileName)
{

    String content = readFile(fileName);

    if(DEBUG_SD)
    {
        Serial.print(F("File content: "));
        Serial.println(content);
    }

    this->cd("..");
    this->cd("LOG");
    this->createFile(fileName);
    this->writeLine(fileName, content);
    this->cd("..");
    this->cd("TMP");
    return true;
}

void SdIstsos::cd(const String& dirName)
{
    this->sendCommand("cd " + dirName);
    this->waitToCommand();
    delay(5);
}

void SdIstsos::waitToCommand()
{
    while(1)
    {
        if(this->openLog->available())
        {
            if(this->openLog->read() == '>')
            {
                this->command = true;
                break;
            }
        }
    }
}

void SdIstsos::waitForEdit()
{
    while(1)
    {
        if(this->openLog->available())
        {
            if(this->openLog->read() == '<')
            {
                this->command = false;
                break;
            }
        }
    }
}
