/*
 * Formatter.hpp
 *
 *  Created on: Jan 19, 2023
 *      Author: cre
 */

#pragma once

#include "Logger.hpp"

namespace logging
{

class Formatter
{
    public:
        Formatter ();
        virtual ~Formatter () = default;
        virtual void format (char *buffer, const size_t buffer_size, const Logger *const logger, const Level level,
                const int line, const char *const message);
};

}
