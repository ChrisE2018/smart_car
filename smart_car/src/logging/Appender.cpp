/*
 * Appender.cpp
 *
 *  Created on: Jan 5, 2023
 *      Author: cre
 */

#include "Appender.hpp"

Appender::Appender (const Level level, Formatter &formatter) :
        level(level), formatter(formatter)
{
}

Level Appender::get_level () const
{
    return level;
}

void Appender::set_level (const Level _level)
{
    level = _level;
}

void Appender::append (const Logger *logger, const Level level, const int line, const char *message)
{
    formatter.format(buffer, buffer_size, logger, level, line, message);
    buffer[buffer_size - 1] = '\0';
    append(level, buffer);
}
