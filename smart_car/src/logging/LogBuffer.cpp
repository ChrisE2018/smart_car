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

LogBuffer::LogBuffer ()
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

