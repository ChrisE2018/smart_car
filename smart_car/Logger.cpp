/*
 * Logger.cpp
 *
 *  Created on: Dec 27, 2022
 *      Author: cre
 */

#include "Logging.hpp"
#include "Logger.hpp"
#include <stdio.h>

Logger::Logger (Logging &logging, const String name) : logging(logging), name(name)
{
}

const String Logger::get_category ()
{
    return name;
}

void Logger::set_level (Level _level)
{
    level = _level;
}

bool Logger::is_enabled (Level _level)
{
    return _level <= level;
}

void Logger::log (const char *file, int line, Level _level, const char *format, ...)
{
    if (_level <= level)
    {
        va_list args;

        va_start(args, format);
        char buf[buffer_size];
        vsnprintf(buf, buffer_size, format, args);
        logging.log(_level, file, line, this, buf);
        va_end(args);
    }
}
