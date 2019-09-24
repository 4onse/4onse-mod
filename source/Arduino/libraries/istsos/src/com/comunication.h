#ifndef COMUNICATION_H
#define COMUNICATION_H


class ICom
{
    public:
        /**
            initialize comunication

            @return uint8_t return code
        */
        virtual uint8_t begin() = 0;

        /**
            Execute POST requests.

            @param server server to do the request
            @param resource resource URI
            @param message post body

            @return uint8_t return code
        */
        virtual uint8_t executePost(const char server[], const char uri[], const String& data) = 0;
        /**
            Update the internal RTC and return the current date.

            @param ntpServer[] server url for ntp service
            @param GMT timezone to add

            @return uint32_t* array of date
        */
        virtual uint32_t* ntpUpdate(const char ntpServer[] = "", int GMT = 0) = 0;
        //virtual bool getStatus() = 0;
        //virtual bool getCheck() = 0;

        virtual void disconnect() = 0;

        virtual uint8_t connect() = 0;

        virtual void useAutoDisconnect(bool status) = 0;

        virtual void useSSL(bool flag) = 0;
};

#endif
