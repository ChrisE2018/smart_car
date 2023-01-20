/*
 * Logger.cpp
 *
 *  Created on: Jan 2, 2023
 *      Author: cre
 */

#include "Appender.hpp"
#include "Logger.hpp"
#include "LogBuffer.hpp"

#include <WString.h>

static Logger *Logger::ROOT = new Logger(nullptr, "root", Level::debug);

static char Logger::buffer[Logger::buffer_size];

static LogBuffer Logger::stream;

Logger::Logger (const String name) :
        Logger(Logger::ROOT, name, Level::info)
{
}

Logger::Logger (const String name, const Level level) :
        Logger(Logger::ROOT, name, level)
{
}

Logger::Logger (Logger *parent, const String name, const Level level) :
        parent(parent), name(name), short_name(shorten(name)), level(level)
{
}

const String Logger::get_name () const
{
    return name;
}

const String Logger::get_short_name () const
{
    return short_name;
}

const String Logger::shorten (const String name)
{
    const String &filename = name;
    const int slash_pos = filename.lastIndexOf('/');
    if (slash_pos >= 0)
    {
        filename = filename.substring(slash_pos + 1);
    }
    const int dot_pos = filename.lastIndexOf('.');
    if (dot_pos >= 0)
    {
        filename = filename.substring(0, dot_pos);
    }
    return filename;
}

const Level Logger::get_level () const
{
    return level;
}

void Logger::add_appender (Appender *const appender)
{
    if (appender != nullptr)
    {
        if (std::find(appenders.begin(), appenders.end(), appender) == appenders.end())
        {
            appenders.push_back(appender);
        }
    }
}

LogBuffer& Logger::info ()
{
    stream.set_logger(this);
    stream.set_level(Level::info);
    stream.set_line(0);
    return stream;
}

LogBuffer& Logger::debug ()
{
    stream.set_logger(this);
    stream.set_level(Level::debug);
    stream.set_line(0);
    return stream;
}

LogBuffer& Logger::data ()
{
    stream.set_logger(this);
    stream.set_level(Level::data);
    stream.set_line(0);
    return stream;
}

LogBuffer& Logger::info (const int line)
{
    stream.set_logger(this);
    stream.set_level(Level::info);
    stream.set_line(line);
    return stream;
}

LogBuffer& Logger::debug (const int line)
{
    stream.set_logger(this);
    stream.set_level(Level::debug);
    stream.set_line(line);
    return stream;
}

void Logger::logging (const Level _level, const int line, const char *format, ...)
{
    if (static_cast<int>(_level) <= static_cast<int>(level))
    {
        va_list args;
        va_start(args, format);
        vsnprintf(buffer, buffer_size, format, args);
        append(this, _level, line, buffer);
        va_end(args);
    }
}

void Logger::logging_p (const Level _level, const int line, const char *format, ...)
{
    if (static_cast<int>(_level) <= static_cast<int>(level))
    {
        va_list args;
        va_start(args, format);
        vsnprintf_P(buffer, buffer_size, format, args);
        append(this, _level, line, buffer);
        va_end(args);
    }
}

void Logger::append (const Logger *logger, const Level level, const int line, const char *message)
{
    for (Appender *appender : appenders)
    {
        appender->append(logger, level, line, message);
    }
    if (parent != nullptr)
    {
        parent->append(logger, level, line, message);
    }
}
