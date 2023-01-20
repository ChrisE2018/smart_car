/*
 * StandardFormatter.cpp
 *
 *  Created on: Jan 19, 2023
 *      Author: cre
 */

#include "StandardFormatter.hpp"

namespace logging
{

StandardFormatter::StandardFormatter (TimeSource &time_source) :
        Formatter(), time_source(time_source)
{
}

void StandardFormatter::format (char *buffer, const size_t buffer_size, const Logger *const logger, const Level level,
        const int line, const char *const message)
{
    const time_t t = time_source.get_unixtime();
    struct tm *const lt = localtime(&t);
    const int ms = millis() % 1000;
    snprintf(buffer, buffer_size, "%s.%03d [%s %s:%d] %s", isotime(lt), ms, stringify(level),
            logger->get_short_name().c_str(), line, message);
    buffer[buffer_size - 1] = '\0';
}

}
