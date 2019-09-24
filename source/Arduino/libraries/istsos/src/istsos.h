#ifndef ISTSOS_H
#define ISTSOS_H

#include "log/log.h"
#include "com/comunication.h"
#include "utils.h"

#define MAX_RETRY 3

// enable/disable debug log
#define DEBUG_ISTSOS

class Istsos
{
    private:
        ILog* sd;
        ICom* com;
        const char* server;
        const char* resource;
        const char* procedure;

        uint8_t sendSingleFile(const String& fileName);


    public:

        /**
            constructor.

            @param sd
            @param com
            @param server
            @param resource
            @param procedure

        */
        Istsos(ILog &sd, ICom &com, const char* server, const char* resource, const char* procedure);

        /**
            initialize SD, call it inside setup().
        */
        uint8_t beginSD();

        /**
            initialize SIM800, call it inside setup().
        */
        uint8_t beginSIM800();

        /**
            Send available data to the server (testing).

            @return bool success or failure
        */
        uint8_t sendData();

        /**
            Log message to the SD card.

            @param message message to log
        */
        void logData(const String& message);
        /**
            this function log some message i.e. last send date.

            @param message massage to save
        */
        void logging(const String& date, const String& message);

        /**
            Get current time and date

            @param ntpServer
        */
        uint32_t* ntpUpdate(const char ntpServer[] = "", int GMT = 0);

        /**
        *   Check if the temporary folder contains data
        *
        *   @return bool true if tmp data are present
        */
        bool checkMissingData();

};

#endif
