/*
 * Serial2Appender.cpp
 *
 *  Created on: Jan 20, 2023
 *      Author: cre
 */

#include "Serial2Appender.hpp"

namespace logging
{

Serial2Appender::Serial2Appender (const Level level, Formatter &formatter) :
        Appender(level, formatter)
{
}

void Serial2Appender::append (const Level _level, const char *message)
{
    if (static_cast<int>(_level) <= static_cast<int>(level))
    {
        Serial2.println(message);
    }
}

}
