/*
 * Appender.hpp
 *
 *  Created on: Jan 5, 2023
 *      Author: cre
 */

#pragma once

#include "Logger.hpp"

class Appender
{
    public:
        Appender ();
        virtual ~Appender () = default;
        virtual void append (const Logger *logger, const Level level, const int line,
                const char *message);
};

