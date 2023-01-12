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

static char LogBuffer::buffer[LogBuffer::buffer_size];

static int LogBuffer::pos = 0;

std::ostream& operator<< (std::ostream &lhs, const __FlashStringHelper *pstr)
{
    PGM_P p = reinterpret_cast<PGM_P>(pstr);
    while (true)
    {
        unsigned char c = pgm_read_byte(p++);
        if (c == 0)
        {
            return lhs;
        }
        lhs.put(c);
    }
}

LogBuffer::LogBuffer (Logger *logger, const Level level) :
                logger(logger), level(level), std::ios(0), std::ostream(this)
{
}

std::streambuf::int_type LogBuffer::overflow (const std::streambuf::int_type c)
{
    if (c == '\n' || c == EOF || pos + 1 >= buffer_size)
    {
        flush();
    }
    else if (c != '\r')
    {
        buffer[pos++] = c;
    }
    buffer[pos] = '\0';
    return c;
}

void LogBuffer::flush ()
{
    logger->append(logger, level, line, buffer);
    reset();
}

void LogBuffer::reset ()
{
    pos = 0;
    buffer[pos] = '\0';
}

void LogBuffer::set_line (const int _line)
{
    line = _line;
}
