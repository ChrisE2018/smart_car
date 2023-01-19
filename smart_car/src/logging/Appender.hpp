/*
 * Appender.hpp
 *
 *  Created on: Jan 5, 2023
 *      Author: cre
 */

#pragma once

#include "Logger.hpp"
#include "Formatter.hpp"

class Appender
{
    public:
        Appender (const Level level, Formatter &formatter);
        virtual ~Appender () = default;
        Level get_level () const;
        void set_level (const Level level);
        virtual void append (const Logger *logger, const Level level, const int line, const char *message);
        virtual void append (const Level level, const char *message);

    private:
        Level level;
        static const int buffer_size = 128;
        char buffer[buffer_size];
        Formatter &formatter;
};

