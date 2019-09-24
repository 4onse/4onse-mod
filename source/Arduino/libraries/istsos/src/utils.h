
#ifndef UTILS_H
#define UTILS_H

#include "Arduino.h"

int getArraySize(const String& message, const char separator);
String getValue(const String& data, const char separator, int index);

#endif
