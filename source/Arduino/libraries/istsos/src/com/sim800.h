#ifndef ISTSOSGPRS_H
#define ISTSOSGPRS_H

#define SIM_POWER 40

#define TIMEOUT 60000UL
#define RESPOSE_TIMEOUT 60000UL


// error code
#define REQUEST_SUCCESS 0
#define REQUEST_FAILURE 1
#define CONNECTION_PROBLEM 2
#define NETWORK_FAILURE 3
#define GPRS_FAILURE 4

// enable/disable debug
// #define DEBUG_COM
// #define DEBUG_COM_ANS

#define OK "OK"
#define ERROR "ERROR"
#define TMP_REG_OK_HOME 1
#define TMP_REG_OK_ROAMING 5


#include "utils.h"
#include "comunication.h"

class Sim800 : public ICom
{

private:
    // generic for HardwareSerial or SoftwareSerial
    Stream* serialAT;

    const char* apn;
    const char* user;
    const char* pass;
    const char* pin;
    const char* basic;

    bool ssl;
    bool autoDisconnect;

    /**
        Get shield status

        @return bool status
    */
    bool getStatus();

    /**
        Read response from the server
    */
    bool getResponse();
    /**
        Wake up shield if it's in sleep mode.
    */
    void wakeUpShield();
    /**
        Put the shield in sleep mode.
    */
    void sleepMode();
    // /**
    //     Get shield status

    //     @return bool status
    // */
    // bool getStatus();

    /**
        To
    */
    void power();
    /**
        Unlock sim.

    */
    bool unlockSim();

    /**
        Send command and wait for response

        @param command  command to be executed
        @param timeout  limit to wait for a response
        @param expected expected response
    */
    template<typename T>
    uint8_t sendCmd(T command, uint32_t timeout=60000UL, const String expected="");

    /**
        Wait for GPRS to connect to the network

        @param timeout  max timeout

        @return bool    true if connected
    */
    bool waitForNetwork(unsigned long timeout=30000UL);

    /**
        Connect to the APN
    */
    bool connectToNetwork();
    /**
        Check if connected to the network
    */
    uint8_t getRegStatus();

    /**
        Wait for a response

        @param timeout  max timeout
        @param expected Expected string to wait

        @return true if expected string or OK
    */
    uint8_t waitResponse(uint32_t timeout=60000UL, const String expected="");

    /**
        Restart modem

        @param bool true if restarted
    */
    bool restart();

    /**
        Set bautrate

        @param timeout timeout

        @return bool
    */
    bool autoBaud(unsigned long timeout = 10000UL);

    /**
        Send command to modem

        @param comm     command
        @params command list of commands
    */
    template<typename T, typename... Args>
    void writeCmd(T comm, Args... command);

    /**
        Send single command to the modem
        @params command command
    */
    template<typename T>
    void writeCmd(T command);

    /**
        Clear buffer
    */
    void serialFlush();

public:

    /**
        Constructor with Stream.

        @param serial   Hardware or Software serial
        @param apn      Access Point Name ("gprs.swisscom.ch")
        @param user     APN user ("gprs")
        @param pass     APN password ("gprs")
        @param basic    Basic authentication string ("YORhaF92LrYxAPp0XlB?")
        @param pin      pin to unlock the SIM ("1234")
    */
    Sim800(Stream &serial, const char* apn, const char* user, const char* pass, const char* basic="", const char* pin="");

    /**
        Initialize SIM enviroment.
        Call it inside setup()

        @return bool TODO
    */
    uint8_t begin();

    /**
        toggle use of ssl

        @param flag true to enable ssl
    */
    void useSSL(bool flag);

    void useAutoDisconnect(bool status);
    /**
        Execute POST requests.

        @param server   server to do the request
        @param resource resource URI
        @param message  post body

        @return uint8_t return code
    */
    uint8_t executePost(const char server[], const char uri[], const String& data);

    /**
        Update the internal RTC and return the current date.

        @return int* array of date [year, month, day, hour, minute, second, tz hour, tz min]
    */
    uint32_t* ntpUpdate(const char ntpServer[], int GMT);


    /**
        Disconnect from GPRS network.
    */
    void disconnect();

    /**
    */
    uint8_t connect();
};
#endif
