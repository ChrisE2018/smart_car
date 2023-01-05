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
#include "Logger.hpp"

static char LogBuffer::buffer[LogBuffer::buffer_size + 1];

static int LogBuffer::pos = 0;

LogBuffer::LogBuffer (Logger *logger, const Level level) : logger(logger), level(level), std::ios(
        0), std::ostream(this)
{
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

void LogBuffer::flush ()
{
    logger->append(logger, level, 0, buffer);
    reset();
}

void LogBuffer::reset ()
{
    pos = 0;
    buffer[pos] = '\0';
}

