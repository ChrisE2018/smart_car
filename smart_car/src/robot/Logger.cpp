/*
 * Logger.cpp
 *
 *  Created on: Jan 2, 2023
 *      Author: cre
 */

#include "Logger.hpp"

#include <WString.h>

const char* stringify (const Level level)
{
    switch (level)
    {
        case Level::error:
            return "error";
        case Level::warning:
            return "warning";
        case Level::info:
            return "info";
        case Level::debug:
            return "debug";
        default:
            return "?level?";
    }
}

Logger::Logger (const String name) : name(name), level(Level::info)
{
}

Logger::Logger (const String name, const Level level) : name(name), level(level)
{
}

const String Logger::get_name () const
{
    return name;
}

void Logger::set_level (const Level _level)
{
    level = _level;
}

const Level Logger::get_level () const
{
    return level;
}

void Logger::logging (const Level level, const int line, const char *format, ...)
{
    va_list args;

    va_start(args, format);
    char buf[buffer_size];
    const char * lvl = stringify(level);
    const int n = snprintf(buf, buffer_size, "%s [%s] ", name.c_str(), lvl);
    vsnprintf(buf + n, buffer_size - n, format, args);
    Serial.println(buf);
    Serial1.println(buf);
    va_end(args);
}
