#ifndef SDISTSOS_H
#define SDISTSOS_H

#include "Arduino.h"
#include "utils.h"
#include "log.h"

#define RESET_OPENLOG 36

#define DEBUG_SD 0

class SdIstsos : public ILog
{
    private:

        Stream* openLog;
        bool command = false;
        void gotoCommandLine();

        void waitToCommand();
        void waitForEdit();

        void sendCommand(const String& command);

        template<typename T, typename... Args>
        void writeCommand(T com, Args... command);

        template<typename T>
        void writeCommand(T com);

        template<typename... Args>
        void sendCommandTest(Args... commands);

        void clearBuffer();
        // int lastChar = 0;
        // String lastFile = "";
        bool reset();

    public:
        /**
            Class to manage sd card logging.

            @param serial serial to send command to OpenLog
        */
        SdIstsos(Stream &serial);

        /**
            Create a new file.
            @param fileName name of the new file
        */
        bool createFile(const String& fileName);
        /**
            Read the content of a file.

            @param fileName name of the file to read
            @return file content
        */
        String readFile(const String& fileName);
        /**
            Funtion to initialize the openLog, call it inside setup().
        */
        bool begin();

        bool init();
        /**
            Write line to selected file.
            If the file exists it will append to the end of the file.
            If the file doesn't exist a new file is created.

            @param file name of the file
            @param line line to write to the file
        */
        void writeLine(const String& fileName, const String& line);
        /**
            Delete selected file

            @param file filename to remove
        */
        void removeFile(const String& fileName);

        /**
            Get file size.

            @param file name of the file
            @return  size of the file in byte
        */
        int getFileSize(const String& fileName);

        /**
            Move selected file from TMP folder to LOG folder.

            @param fileName name of the file to move.
            @return boolean
        */
        bool copyToLog(const String& fileName);

        /**
            List files inside current directory
        */
        String ls();

        /**
            TODO
        */
        bool openFile(const String& fileName);
        /**
            Read new line from file.
            Every time that you call this function is returned a new line.

            @param file fileName
            @return line as String
        */
        String readLine(const String& file);

        /**
            Create new direcotry.

            @param dirName name of the new directory
            @return TODO
        */
        bool createDirectory(const String& dirName);
        /**
            Alias to changeDir method.

            @param dirName name of the directory
        */
        void cd(const String& dirName);

        String getFirstFileName();

};

#endif
