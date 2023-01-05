/*
 * Logger.cpp
 *
 *  Created on: Jan 2, 2023
 *      Author: cre
 */

#include "Appender.hpp"
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

static Logger *Logger::ROOT = new Logger(nullptr, "root", Level::debug);

//LoggerOstream::LoggerOstream (Level level, Logger *logger) : level(level), logger(logger), std::ios(
//        0), std::ostream((LogBuffer*) this)
//{
//}
//
//void LoggerOstream::flush ()
//{
//    const std::string &buf = LogBuffer::get_buffer();
//    logger->append(logger, level, 0, buf.c_str());
//}

// http://www.angelikalanger.com/Articles/C++Report/IOStreamsDerivation/IOStreamsDerivation.html
// https://gcc.gnu.org/onlinedocs/libstdc++/manual/streambufs.html
Logger::Logger (const String name) : parent(Logger::ROOT), name(name), short_name(shorten(name)), level(
        Level::info), info_stream(this, Level::info)
{
}

//Logger::Logger (const String name) : parent(Logger::ROOT), name(name), short_name(shorten(name)), level(
//        Level::info), error_stream(this, Level::error), warning_stream(this, Level::warning), info_stream(
//        this, Level::info), debug_stream(this, Level::debug)
//{
//}

Logger::Logger (const String name, const Level level) : Logger(Logger::ROOT, name, level)
{
}

Logger::Logger (Logger *parent, const String name, const Level level) : parent(parent), name(name), short_name(
        shorten(name)), level(level), info_stream(this, Level::info)
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

void Logger::set_level (const Level _level)
{
    level = _level;
}

const Level Logger::get_level () const
{
    return level;
}

void Logger::add_appender (Appender *appender)
{
    appenders.push_back(appender);
}

//LogBuffer& Logger::error ()
//{
//    return error_stream;
//}
//
//LogBuffer& Logger::warning ()
//{
//    return warning_stream;
//}

LogBuffer& Logger::info ()
{
    return info_stream;
}

//LogBuffer& Logger::debug ()
//{
//    return debug_stream;
//}

void Logger::logging (const Level _level, const int line, const char *format, ...)
{
    if (static_cast<int>(_level) <= static_cast<int>(level))
    {
        va_list args;
        va_start(args, format);
        char buf[buffer_size];
        vsnprintf(buf, buffer_size, format, args);
        append(this, level, line, buf);
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
