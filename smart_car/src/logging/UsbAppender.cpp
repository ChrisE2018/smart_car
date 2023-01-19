/*
 * UsbAppender.cpp
 *
 *  Created on: Jan 19, 2023
 *      Author: cre
 */

#include "UsbAppender.hpp"

UsbAppender::UsbAppender (Formatter &formatter, const Level level) :
        Appender(formatter), level(level)
{
}

void UsbAppender::append (const Level _level, const char *message)
{
    if (static_cast<int>(_level) <= static_cast<int>(level))
    {
        Serial.println(message);
    }
}
