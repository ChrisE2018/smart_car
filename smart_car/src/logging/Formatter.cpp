/*
 * Formatter.cpp
 *
 *  Created on: Jan 19, 2023
 *      Author: cre
 */

#include "Formatter.hpp"

Formatter::Formatter ()
{
}

void Formatter::format (char *buffer, const size_t buffer_size, const Logger *const logger, const Level level,
        const int line, const char *const message)
{
    strlcpy(buffer, message, buffer_size);
}
