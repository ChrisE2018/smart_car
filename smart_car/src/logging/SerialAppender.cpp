/*
 * SerialAppender.cpp
 *
 *  Created on: Jan 19, 2023
 *      Author: cre
 */

#include "SerialAppender.hpp"

SerialAppender::SerialAppender (const Level level, Formatter &formatter) :
        Appender(level, formatter)
{
}

void SerialAppender::append (const Level _level, const char *message)
{
    if (static_cast<int>(_level) <= static_cast<int>(level))
    {
        Serial.println(message);
    }
}
