/*
 * StandardFormatter.hpp
 *
 *  Created on: Jan 19, 2023
 *      Author: cre
 */

#pragma once

#include "Logger.hpp"
#include "Formatter.hpp"
#include "TimeSource.hpp"

class StandardFormatter : public Formatter
{
    public:
        StandardFormatter (TimeSource &time_source);
        virtual ~StandardFormatter () = default;
        virtual void format (char *buffer, const size_t buffer_size, const Logger *const logger, const Level level,
                const int line, const char *const message);

    private:
        TimeSource &time_source;
};

