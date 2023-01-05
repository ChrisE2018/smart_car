/*
 * LogBuffer.cpp
 *
 *  Created on: Jan 5, 2023
 *      Author: cre
 */

#include "LogBuffer.hpp"
#include <locale>
#include <cstdio>
#include <streambuf>
#include <Arduino.h>
#include "Logger.hpp"

LogBuffer::LogBuffer (Logger *logger, const Level level) : logger(logger), level(level), std::ios(
        0), std::ostream(this)
{
}

std::string LogBuffer::get_buffer ()
{
    std::string result = std::string(buffer);
    reset();
    return result;
}

void LogBuffer::reset ()
{
    pos = 0;
    buffer[pos] = '\0';
}

void LogBuffer::flush ()
{
    const std::string &buf = LogBuffer::get_buffer();
    logger->append(logger, level, 0, buf.c_str());
}

std::streambuf::int_type LogBuffer::overflow (std::streambuf::int_type c)
{
    if (c == '\n' || c == EOF || pos >= buffer_size)
    {
        flush();
    }
    else
    {
        buffer[pos++] = c;
    }
    buffer[pos] = '\0';
    return c;
}

