/*
 * LogBuffer.cpp
 *
 *  Created on: Jan 5, 2023
 *      Author: cre
 */

#include <locale>
#include <cstdio>
#include <streambuf>
#include "LogBuffer.hpp"
#include "Logger.hpp"

namespace logging
{

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

LogBuffer::LogBuffer () :
        logger(nullptr), level(Level::info), std::ios(0), std::ostream(this)
{
}

std::streambuf::int_type LogBuffer::overflow (const std::streambuf::int_type c)
{
    if (c == '\n' || c == EOF || Logger::pos + 1 >= Logger::buffer_size)
    {
        flush();
    }
    else if (c != '\r')
    {
        Logger::buffer[Logger::pos++] = c;
    }
    Logger::buffer[Logger::pos] = '\0';
    return c;
}

void LogBuffer::flush ()
{
    logger->append(logger, level, line, Logger::buffer);
    reset();
}

void LogBuffer::reset ()
{
    Logger::pos = 0;
    Logger::buffer[0] = '\0';
}

void LogBuffer::set_logger (Logger *const _logger, const Level _level, const int _line)
{
    logger = _logger;
    level = _level;
    line = _line;
}

}
