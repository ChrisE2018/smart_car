/*
 * Serial1Appender.cpp
 *
 *  Created on: Jan 20, 2023
 *      Author: cre
 */

#include "Serial1Appender.hpp"

namespace logging
{

Serial1Appender::Serial1Appender (const Level level, Formatter &formatter) :
        Appender(level, formatter)
{
}

void Serial1Appender::append (const Level _level, const char *message)
{
    if (static_cast<int>(_level) <= static_cast<int>(level))
    {
        Serial1.println(message);
    }
}

}
