/*
 * Appender.cpp
 *
 *  Created on: Jan 5, 2023
 *      Author: cre
 */

#include "Appender.hpp"

Appender::Appender (Formatter &formatter) :
        formatter(formatter)
{
}

void Appender::append (const Logger *logger, const Level level, const int line, const char *message)
{
    formatter.format(buffer, buffer_size, logger, level, line, message);
    buffer[buffer_size - 1] = '\0';
    append(level, buffer);
}
