/*
 * Appender.cpp
 *
 *  Created on: Jan 5, 2023
 *      Author: cre
 */

#include "Appender.hpp"

Appender::Appender ()
{
}

void Appender::set_formatter (Formatter *const _formatter)
{
    formatter = _formatter;
}

void Appender::append (const Logger *logger, const Level level, const int line, const char *message)
{
    if (formatter == nullptr)
    {
        append(level, message);
    }
    else
    {
        formatter->format(buffer, buffer_size, logger, level, line, message);
        append(level, buffer);
    }
}
