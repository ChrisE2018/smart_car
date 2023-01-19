/*
 * UsbAppender.cpp
 *
 *  Created on: Jan 19, 2023
 *      Author: cre
 */

#include "UsbAppender.hpp"

UsbAppender::UsbAppender (const Level level, Formatter &formatter) :
        Appender(level, formatter)
{
}

void UsbAppender::append (const Level _level, const char *message)
{
    if (static_cast<int>(_level) <= static_cast<int>(get_level()))
    {
        Serial.println(message);
    }
}
