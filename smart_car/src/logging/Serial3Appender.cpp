/*
 * Serial3Appender.cpp
 *
 *  Created on: Jan 20, 2023
 *      Author: cre
 */

#include "Serial3Appender.hpp"

Serial3Appender::Serial3Appender (const Level level, Formatter &formatter) :
        Appender(level, formatter)
{
}

void Serial3Appender::append (const Level _level, const char *message)
{
    if (static_cast<int>(_level) <= static_cast<int>(level))
    {
        Serial3.println(message);
    }
}

