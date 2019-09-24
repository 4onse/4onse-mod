#ifndef SD_H
#define SD_H

#include "Arduino.h"

class ILog
{
    public:
        /**
            Initialize logger.

            @return success or failure
        */
        virtual bool begin() = 0;
        /**
            Funtion to reinit the SD. Helpful if the SD stop responding.
        */
        virtual bool init() = 0;
        /**
            Create new file.

            @param fileName name of the new file

            @return success or failure
        */
        virtual bool createFile(const String& fileName) = 0;
        /**
            Open file on append mode.

            @param fileName namo of the file

            @return success or failure
        */
        virtual bool openFile(const String& fileName) = 0;
        /**
            Write single line to selected file. THis function open the file, write the line and close it.

            @param filName name og the file
            @param line message to write to the file
        */
        virtual void writeLine(const String& fileName, const String& line) = 0;

        /**
            Read file content.

            @param fileName name of the file
            @return file content
        */
        virtual String readFile(const String& fileName) = 0;
        /**
            Delete a file.

            @param fileName name of the file to remove
        */
        virtual void removeFile(const String& fileName) = 0;
        /**
            Get file size.

            @param fileName name ogf the file
            @return file size
        */
        virtual int getFileSize(const String& fileName) = 0;
        /**
            List files.

            @return String with comma separated filenames
        */
        virtual String ls() = 0;
        /**
            Create new directory.

            @param dirName name of the direcotry
        */
        virtual bool createDirectory(const String& dirName) = 0;
        /**
            Change directory. Change working directory.

            @param dirName name of the directory
        */
        virtual void cd(const String& dirName) = 0;
        /**
            Copy file from TMP floder to LOG folder.

            @param fileName name of the file.
            @return success or failure
        */
        virtual bool copyToLog(const String& fileName) = 0;
        /**
            Get name of the first file.

            @return name of the file
        */
        virtual String getFirstFileName() =0;
};
#endif
