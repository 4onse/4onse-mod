
#include "utils.h"

int getArraySize(const String& message, const char separator)
{
    int length = message.length();
    if (length == 0)
    {
        return 0;
    }

    int count = 0;
    for (int i = 0; i < length; i++)
    {
        if (message.charAt(i) == separator)
        {
            count++;
        }

    }
    return ++count;
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
